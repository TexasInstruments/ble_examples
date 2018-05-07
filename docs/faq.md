## Frequently Asked Questions

* [What if I installed the SimpleLink CC26x2 SDK at a non default location (e.g. Not at ``C:\ti\``)?](#what-if-i-installed-the-simplelink-cc26x2-sdk-at-a-non-default-location-eg-not-at-cti)
* [How can I setup my toolchain for use with the ble\_examples repo?](#how-can-i-setup-my-toolchain-for-use-with-the-ble_examples-repo)
* [I am using a LaunchPad device, how can I view display data?](#i-am-using-a-launchpad-device-how-can-i-view-display-data)
---

### <a name="default"></a>What if I installed the SimpleLink CC26x2 SDK at a non default location (e.g. Not at ``C:\ti\``)?

All projects reference files from the BLE stack using environment variables, you
can change this in your IDE's project files.

**CCS**

No changes to the project is necessary. You just need to ensure that Code
Composer Studio was able to discover the SimpleLink CC26x2 SDK in the
available *RTSC Products* tab. See the [BLE5-Stack User's Guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC26X2%20SDK%2FDocuments%2FBLE5-Stack%2FBLE5-Stack%20User's%20Guide)
for more details.

**IAR**

1. Navigate to the sample project directory within the repo and open the
   `.custom_argvars` file.
1. Update the value of ``SIMPLELINK_CORE_SDK_INSTALL_DIR`` to point to your
   custom installation directory where you installed the SimpleLink CC26x2 SDK.

---
### <a name="versioning"></a>How should I keep my application based on a TI BLE-Stack SDK under version control?

There are many ways to solve and address this issue. We are merely presenting
one suggested workflow out of the many combinations that exist.

Since this is an advanced topic, it has its own page dedicated to it.
You can read more here: [Version Control](suggested_workflow.md)

---

### <a name="toolchain"></a>How can I setup my toolchain for use with the ble\_examples repo?

**CCS**

For BLE5-Stack examples, see the *Using BLE5-Stack Projects with CCS* section in the
[BLE5-Stack Quick Start Guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC26X2%20SDK%2FDocuments%2FBLE5-Stack%2FBLE5-Stack%20Quick%20Start%20Guide)

**IAR**

For BLE5-Stack examples, see the *Using BLE5-Stack Projects with IAR* section in the
[BLE5-Stack Quick Start Guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC26X2%20SDK%2FDocuments%2FBLE5-Stack%2FBLE5-Stack%20Quick%20Start%20Guide)

---

### <a name="display"></a>I am using a LaunchPad device, how can I view display data?
LaunchPad projects are already set up to use the tidriver Display. For more
information about the Display driver please see the Display.h driver
documentation in the SimpleLink CC26x2 SDK (`docs/tidrivers/tidriversAPI.html`)

To setup your PC to receive this data please follow the steps below:

1. Install PuTTY or another serial terminal emulator
1. Use the Windows Device Manager (Start &rarr; Run &rarr; `mmc devmgmt.msc`
    &rarr; Ok) to determine which COM port you should connect to:

    ![Device Manager](resources/dev_mgr_xds110.png)
    * Note the COM port number of the `XDS110 Class Application/User UART` listed.
1. Configure putty as a serial console with 115200 8N1.
1. Fill in the Serial Line field with the COM port from above.
1. When configured correctly, the program should look as below:


| Main Screen                                   | Serial Screen                         |
|:---------------------------------------------:|:-------------------------------------:|
| ![Putty Main](resources/putty_main_scrn.png)  | ![Putty serial](resources/putty_serial_scrn.png) |

**You may need to unplug/replug your LaunchPad and restart PuTTY if you do not see any output.**
