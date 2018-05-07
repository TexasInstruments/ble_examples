---
# i2secho

---

## Example Summary

Example that uses the I2S driver to read and write audio frames from an
external codec on the CC3200AUDBOOST.

## Peripherals Exercised

* `Board_I2C0`  - Used to control and setup the TLV320AIC3254 on the
                  CC3200AUDBOOST
* `I2S0`        - Used input and output PCM samples from the CC3200AUDBOOST.

## Resources & Jumper Settings

> If you're using an IDE (such as CCS or IAR), please refer to Board.html in
your project directory for resources used and board-specific jumper settings.
Otherwise, you can find Board.html in the directory
&lt;SDK_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.

## Required Hardware

* [CC26X2R1 LaunchPad](http://www.ti.com/tool/LAUNCHXL-CC26X2R1).
* [CC3200AUDBOOST](http://www.ti.com/tool/CC3200AUDBOOST).

## Example Usage

* Attach the CC3200AUDBOOST to the LaunchPad's BoosterPack connector.

* Connect headphones to the jack labed `LINE OUT` on the CC3200AUDBOOST.
  **Note: apple headphones cannot be used**

* Open a serial session (e.g. [`PuTTY`](http://www.putty.org/ "PuTTY's
Homepage"), etc.) to the appropriate COM port.
    * The COM port can be determined via Device Manager in Windows or via
`ls /dev/tty*` in Linux.

The connection should have the following settings
```
    Baud-rate:  115200
    Data bits:       8
    Stop bits:       1
    Parity:       None
    Flow Control: None
```

* Run the example. Speak into the microphone on the CC3200AUDBOOST. The sound
should be echoed back to the headphones

## Application Design Details

* This example shows how to initialize the I2S driver in callback read
and write mode. The received data is not processed, but instead is sent back
to the to I2S driver to be played back.

* Internally the I2S driver will use DMA

* The `audiocodec.(c|h)` files are used to configure the TLV320AIC3254 with
the parameters needed to support this demo. This is a wrapper on the I2C
register interface needed by this device.

* Currently the application only supports the following audio parameters:

  - 16kHz sample rate
  - 16 bit sample depth

TI-RTOS:

* When building in Code Composer Studio, the kernel configuration project will
be imported along with the example. The kernel configuration project is
referenced by the example, so it will be built first. The "release" kernel
configuration is the default project used. It has many debug features disabled.
These feature include assert checking, logging and runtime stack checks. For a
detailed difference between the "release" and "debug" kernel configurations and
how to switch between them, please refer to the SimpleLink MCU SDK User's
Guide. The "release" and "debug" kernel configuration projects can be found
under &lt;SDK_INSTALL_DIR&gt;/kernel/tirtos/builds/&lt;BOARD&gt;/(release|debug)/(ccs|gcc).

