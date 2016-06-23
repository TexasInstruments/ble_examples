/******************************************************************************

 @file  simple_central.h

 @brief This file contains the Simple BLE Central sample application
        definitions and prototypes.

 Group: WCS, BTS
 $Target Device: DEVICES $

 ******************************************************************************
 $License: BSD3 2013 $
 ******************************************************************************
 $Release Name: PACKAGE NAME $
 $Release Date: PACKAGE RELEASE DATE $
 *****************************************************************************/

#ifndef SPPBLEClient_H
#define SPPBLEClient_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "onboard.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
//LED parameters 
#define Board_LED_TOGGLE                      3
#define BLINK_DURATION                        1000 // Milliseconds
  
 /* Delay */
#ifdef TI_DRIVERS_I2C_INCLUDED
#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
#define MS_2_TICKS(ms) ( ((ms) * 1000) / Clock_tickPeriod )
#else
#define delay_ms(i) ( CPUdelay(12000*(i)) )
#endif
/*********************************************************************
 * MACROS
 */
#if !defined(DEBUG_SIMPLE)
#  define Display_print0(handle, line, col, fmt) DEBUG(fmt); \
    DEBUG_NEWLINE()
#  define Display_print1(handle, line, col, fmt, a0) DEBUG(fmt); \
    uint8 buf[7]; _itoa((uint16)a0, (uint8 *)buf,10); DEBUG(buf); DEBUG_NEWLINE()
#else
#  define Display_print0(handle, line, col, fmt)

#  define Display_print1(handle, line, col, fmt, a0)

#endif
      
#ifdef CC2650_LAUNCHXL
#define Board_LED1 Board_RLED
#define Board_LED2 Board_GLED
#else

#endif //CC2650_LAUNCHXL 
      
/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple BLE Central.
 */
extern void SPPBLEClient_createTask(void);
extern void SPPBLEClient_toggleLed(uint8_t led, uint8_t state);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SPPBLEClient_H */
