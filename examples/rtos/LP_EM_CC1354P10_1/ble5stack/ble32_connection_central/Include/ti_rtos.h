// *****************************************************************************
// ti_rtos.h
// *****************************************************************************

// TI-RTOS header files 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

#if !defined ( DeviceFamily_MSP432 )
#include <ti/drivers/I2S.h>
#endif
////#include <ti/drivers/i2s/I2SCC32XXDMA.h>
//#include <ti/drivers/i2s/I2SCC32XX.h>
#include <ti/drivers/SPI.h>
//#include <ti/drivers/SDSPI.h>
#include <ti/drivers/PWM.h>

#if defined ( DeviceFamily_CC13X4 ) || defined ( USE_UART2 )
    #include <ti/drivers/UART2.h>
#else
    #include <ti/drivers/UART.h>
#endif

//#include <ti/drivers/WiFi.h>
#if defined ( USE_SD_CARD )
#include <ti/drivers/SDFatFS.h>
#endif

// *****************************************************************************
// end of file
// *****************************************************************************
