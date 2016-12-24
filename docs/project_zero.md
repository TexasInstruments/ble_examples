Project Zero
============

### Functional Overview


This project is based on the SimpleBLEPeripheral sample application in the TI BLE SDK.

The following changes are done:
- Import some files to the CCS Workspace instead of linking to filesystem.
- Rename filenames and function names.
- Remove OAD support
- Log to UART instead of writing to display.
- Remove SimpleGATTProfile and use new LED, Button and Data services instead.

A training module based on this project can also be found in SimpleLink Academy. 
Instructions on using Project Zero can be found in the TI BLE Software Developer's Guide.
* [SimpleLink Academy](http://software-dl.ti.com/lprf/simplelink_academy/overview.html)

### Logging

The logging takes up ~23kB of Flash memory in this application, because it uses 
`System_printf` functionality, as well as the `UART` driver and a `Text` table 
from TI-RTOS. In addition there is some overhead per statement, as well as the 
strings themselves.

To **remove** the impact `Log` has on the image size, add a global compiler 
define `xdc_runtime_Log_DISABLE_ALL` as well as changing the TI-RTOS config file 
to read `Text.isLoaded = false;` instead of `true`.

### Demo Requirements

#### Hardware
- 1 CC2650R2 Launchpad
- 1 smart phone

#### Software
- project_zero project from this GIT page
- [SimpleLink CC2640R2 SDK](https://www.ti.com/blestack)
- SimpleLink Starter app (instructions will be given using the SimpleLink Starter 
app but you should be able to use any BLE app with a central mode)

### Running the Demo

1. Compile and load the projects:
    - First build and load `project_zero_cc2640r2lp_stack`
    - Next build and load `project_zero_cc2640r2lp_app`

2. Connect to the LaunchPad via PuTTY (or other serial console emulator). For 
instructions on connecting via PuTTY, please see our [FAQ page](faq.md).

3. After connecting to PuTTY, you will receive log messages relating to the 
initialization procedure and see a message stating the address of the device and 
that it is in the advertising state:

    \#000020 [ 0.014 ] INFO: (../Application/project_zero.c:822) GAP is started. Our address: 0x98072DAA5C67  
    
    \#000021 [ 0.014 ] INFO: (../Application/project_zero.c:827) Advertising  
    
4. Open up the SimpleLink Starter app. Device should be advertising as Project 
Zero R2. 

5. Click the device, then select the Service Explorer option from the pop up menu/
You will notice the PuTTY window has updated to show the GAP state has changed to
Connected and the address of the peer device is listed.

    \#000022 [ 675.689 ] INFO: (../Application/project_zero.c:1315) (CB) GAP State change: 6, Sending msg to app.  
    
    \#000023 [ 675.689 ] INFO: (../Application/project_zero.c:837) Connected. Peer address: 0x5AD80C8EFF73  
    
    \#000024 [ 675.844 ] INFO: (../Application/project_zero.c:1194) MTU Size change: 65 bytes  
    
    
6. The available services should be displayed. Select the service with the UUID that starts
with f0001110 and you should see two Characteristics with Read/Write abilities.
The red LED is controlled by the characteristic with UUID f0001111. The green LED
is controlled by the characteristic with UUID f0001112. Writing a non-zero value
will turn the LEDs on. Writing zero will turn the LEDs off. You will also see
log messages corresponding to your actions in the PuTTY window.
    \#000034 [ 905.071 ] INFO: (../Application/project_zero.c:925) Value Change msg: LED Service LED0: 01  
    
    \#000035 [ 905.071 ] INFO: (../Application/project_zero.c:933) Turning LED0 on  
    
    
7. The button service is controlled by the Service with UUID that starts with f0001120.
Like the LED service, there are two characteristics for the button. The first characteristic
corresponds to BTN-1. The second characteristic corresponds to BTN-2.
To receive updates when a button is pressed, first set the notify state for the characteristic
to 'On'. Then press a button. You should see the value in the Read characteristic window
change between 00 (the button is released) and 01 (the button is pressed). You will
also see corresponding messages in the UART screen.
    \#000131 [ 1341.564 ] INFO: (../Application/project_zero.c:879) Button 0 pressed  
    
    \#000132 [ 1341.564 ] INFO: (../PROFILES/button_service.c:313) SetParameter : BUTTON0 len: 1  
    
    \#000133 [ 1341.564 ] INFO: (../PROFILES/button_service.c:344) Trying to send noti/ind: connHandle 0, Notification enabled  
    
    \#000134 [ 1341.565 ] INFO: (../PROFILES/button_service.c:450) ReadAttrCB : BUTTON0 connHandle: 0 offset: 0 method: 0xff  
    
    \#000135 [ 1342.649 ] INFO: (../Application/project_zero.c:1500) Button interrupt: Button 0  
    
    \#000136 [ 1342.699 ] INFO: (../Application/project_zero.c:879) Button 0 released  
    
    \#000137 [ 1342.699 ] INFO: (../PROFILES/button_service.c:313) SetParameter : BUTTON0 len: 1  
    
    
8. You can send and receive multiple bytes by reading and writing to the first
characteristic of the service with the UUID that starts with f0001130. To send
more than one byte at a time, separate bytes with a comma.




