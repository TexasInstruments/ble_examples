## Frequently Asked Questions

* [What if I installed the SimpleLink CC13XX / CC26XX SDK at a non default location (e.g. Not at ``C:\ti\``)?](#what-if-i-installed-the-simplelink-cc13x2-26x2-sdk-at-a-non-default-location-eg-not-at-cti)
* [How can I setup my toolchain for use with the ble\_examples repo?](#how-can-i-setup-my-toolchain-for-use-with-the-ble_examples-repo)
* [I am using a LaunchPad device, how can I view display data?](#i-am-using-a-launchpad-device-how-can-i-view-display-data)
---

### <a name="default"></a>What if I installed the SimpleLink CC13XX / CC26XX SDK at a non default location (e.g. Not at ``C:\ti\``)?

All projects reference files from the BLE stack using environment variables, you
can change this in your IDE's project files. 

Alternatively, you can make a symlink in the default location pointing to your 
custom location. E.g., if you have cloned this repo to ``C:\git\`` you can use 
the following command:

``mklink /d ble_examples-simplelink_cc13x2_26x2_sdk-6.40 C:\git\ble_examples``

---
### <a name="versioning"></a>How should I keep my application based on a TI BLE-Stack SDK under version control?

There are many ways to solve and address this issue. We are merely presenting
one suggested workflow out of the many combinations that exist.

Since this is an advanced topic, it has its own page dedicated to it.
You can read more here: [Version Control](suggested_workflow.md)

---

### <a name="toolchain"></a>How can I setup my toolchain for use with the ble\_examples repo?

**CCS**

1. Import the project to your CCS workspace
2. Open Project -> Properties -> General -> Products and select the 3.20.00.68 
   version of SimpleLink CC13x2 26x2 SDK.
3. Build and flash the example project.

**IAR**

1. Open IAR, if there is any active workspace close it.
2. Open Tools -> Configure Custom Argvars -> Global and make sure no groups are 
   active (if they are you can disable or delete them with the buttons on the 
   left side). Import ``ble_examples.custom_argvars`` from ``ble_examples\tools\iar``. 
3. Close the Configure Custom Argvars view. Open the ``.template.eww`` file with 
   the Open Workspace option.
4. When asked to select a folder for the workspace, make sure to select an empty 
   folder.
5. Build and flash the example project.

---

### <a name="display"></a>I am using a LaunchPad device, how can I view display data?
LaunchPad projects are already set up to use the tidriver Display. For more
information about the Display driver please see the Display.h driver
documentation in the SimpleLink CC13X2 / CC26x2 SDK (`docs/tidrivers/tidriversAPI.html`)

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
