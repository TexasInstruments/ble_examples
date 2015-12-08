Redirect TI-RTOS Log_xx to UART
========
The logging system in TI-RTOS is quite useful and configurable. In the below we will set the logging system up to give us log events which are stored in a circular buffer and then printed out later in the idle thread.

Log system
----------
The short version is, xdc.runtime.Log provides an interface (a lot of macros) that you can use. The RTOS config file can then be set up to make the RTOS precompiler route the logs to whichever logger implementation you want.

We are going to use xdc.runtime.LoggerCallback, which is a lightweight implementation, to hand us the arguments to the Log call for storage and output.

Implementation
--------------
[LoggerCallback][LoggerCallback] allows us to specify an [outputFxn][outputFxn] which will be called each time `Log_xx` is called. It receives a user-specified argument (ignored in this example), a [Log_EventRec][Log_EventRec] struct which contains the pointers to the strings to be printed, and finally the number of arguments.

To interfere as little as possible with the operation of the device, the `uartLog_outputFxn` does not print anything, just stores the string pointers, and adds a serial number and a timestamp. Each stored record is `40 bytes`, and if more than `UARTLOG_NUM_EVT_BUF` events are received before the device has entered the Idle thread, the oldest record is lost.

In the `Idle` thread, we add `uartLog_flush` which loops through the records in the buffer, formats using System_snprintf into a local buffer of length `UARTLOG_OUTBUF_LEN`, and sends the formatted string array to the UART driver for transport.

**Note**: The UART driver must be opened in **blocking** mode, as the `uartLog_flush` loop depends on this.

Both the above defines for string length and buffer length can be overridden via precompiler defines, or source code changes.


References
----------
* [xdc.runtime.Log cdocs](http://rtsc.eclipse.org/cdoc-tip/xdc/runtime/Log.html)
* [xdc.runtime.LoggerCallback cdocs](http://rtsc.eclipse.org/cdoc-tip/xdc/runtime/LoggerCallback.html)
* [uia.ti.sysbios.LoggerIdle wiki example for Stellaris](http://processors.wiki.ti.com/index.php/LoggerIdle_Uart)
* [RTSC Logging tips](http://rtsc.eclipse.org/docs-tip/Using_xdc.runtime_Logging)

Compiler options
-----------------
### Remove global Log disable define
TI-RTOS will remove all Log_xx statements from compiled code if the define `xdc_runtime_Log_DISABLE_ALL` is defined when `Log.h` is included. Typically, all the BLE SDK example projects have this define present to save flash.
* Remove this define from the project settings, or rename it to e.g. `Xxdc_runtime_Log_DISABLE_ALL`

### Change what `__FILE__` does
`Log_info`, `Log_warning` and `Log_error` includes the filename and line number of the log invocation via `__FILE__` and `__LINE__`. By default the file name includes the complete path to the source file.

This takes up a lot of space in flash and in the UART buffer, so we'll change this.
#### IAR
Add extra compiler option *--no_path_in_file_macros*

#### CCS
Add an extra precompiler define: `xdc_FILE="\"${InputFileName}\""`


TI-RTOS Config file
-------------------
This is where the magic happens. The file is typically called `appBLE.cfg` or something else ending in `.cfg`. We will include the Log module/interface from the XDC Runtime package, and the LoggerCallback plugin for the Log module, and we will tell LoggerCallback to route Log calls to our own output handler function.

```javascript
// Need Text loaded for formatting of Log_info/warning/error, but not for Log_print.
Text.isLoaded = true;

// Logging
var Log = xdc.useModule('xdc.runtime.Log');

// Override error output color with ANSI codes, and use shorter (file.c:line) format.
Log.L_error =  {
    mask: Diags.STATUS,
    level: Diags.ERROR,
    msg: "\x1b[31;1mERROR:\x1b[0m (%s:%d) %$S"
};

Log.L_info = {
    mask: Diags.INFO,
    msg: "\x1b[32;1mINFO:\x1b[0m (%s:%d) %$S"
};

Log.L_warning = {
    mask: Diags.STATUS,
    level: Diags.WARNING,
    msg: "\x1b[33;1mWARNING:\x1b[0m (%s:%d) %$S"
    };

// Pull in LoggerCallback
var LoggerCallback = xdc.useModule('xdc.runtime.LoggerCallback');

// Tell LoggerCallback to call our output function
LoggerCallback.outputFxn = "&uartLog_outputFxn";

// Tell the Idle module to add our flush() function to the idle loop (before Power)
var Idle = xdc.useModule('ti.sysbios.knl.Idle'); // Add if Idle isn't already imported.
Idle.addFunc('&uartLog_flush');

// Create a static instance of LoggerCallback and set as default Main logger
var loggerParams = new LoggerCallback.Params();
loggerParams.arg = 1;

// Only for Main (code that's not in an rtsc module)
Main.common$.logger = LoggerCallback.create(loggerParams);
//Defaults.common$.logger = LoggerCallback.create(loggerParams); // Use for all log events

// Turn on USER1 logs and INFO in Main module (user code). Turn off USER2 for fun.
Main.common$.diags_USER1 = Diags.ALWAYS_ON;
Main.common$.diags_USER2 = Diags.ALWAYS_OFF;
Main.common$.diags_USER6 = Diags.ALWAYS_ON;
Main.common$.diags_INFO = Diags.ALWAYS_ON;
```


main.c or other init
-------------------
Even though our output function is now being called each time Log_xx is called, we need to set up the UART and tell the output function about it.

```c
// To open the UART
#include <ti/drivers/UART.h>
// To initialize the uart log output (only needed here)
#include "uart_logs.h"
// ...
int main()
{
  PIN_init(BoardGpioInitTable);
  // ...
  UART_Params uartParams;
  UART_Params_init(&uartParams);
  uartParams.baudRate = 115200;
  UART_Handle hUart = UART_open(Board_UART, &uartParams);
  // Initialize the logger output
  UartLog_init(hUart);
  // ...
```

Add log statements to your code
-------------------------------

```c
#include <xdc/runtime/Log.h>   // For Log_warning1("Warning number #%d", 4); things
#include <xdc/runtime/Diags.h> // For Log_print0(Diags_USER1, "hello"); things.
// ...
int myfunc()
{
  // ...
  Log_info0("Hello world via Log_info0");
  Log_error0("Out of cheese error!");
}
```

Include uart_logs.c in the project
---------------------
The functions referenced in the TI-RTOS config file, `uartLog_flush` and `uartLog_outputFxn` can be found in [uart_logs.c](uart_logs.c) which must be included.


How it looks
------------
he above example, and Simple BLE Peripheral plus a custom service instrumented with Log_info and Log_error statements looks like this:

![Putty session](http://processors.wiki.ti.com/images/9/9c/Uart_log.png "Putty session")

[LoggerCallback]: http://rtsc.eclipse.org/cdoc-tip/xdc/runtime/LoggerCallback.html
[outputFxn]: http://rtsc.eclipse.org/cdoc-tip/xdc/runtime/LoggerCallback.html#.Output.Fxn
[Log_EventRec]: http://rtsc.eclipse.org/cdoc-tip/xdc/runtime/Log.html#.Event.Rec
