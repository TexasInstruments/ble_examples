/******************************************************************************

 @file  spp_ble_client.h

 @brief This file contains the Simple BLE Central sample application
        definitions and prototypes.

 Group: WCS, BTS
 $Target Device: DEVICES $

 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
#define DEBUG_SIMPLE
  
#if !defined(DEBUG_SIMPLE)
#  define Display_print0(handle, line, col, fmt) DEBUG(fmt); \
    DEBUG_NEWLINE()
#  define Display_print1(handle, line, col, fmt, a0) DEBUG(fmt); \
    uint8 buf[7]; _itoa((uint16)a0, (uint8 *)buf,10); DEBUG(buf); DEBUG_NEWLINE()
#else
#  define Display_print0(handle, line, col, fmt)

#  define Display_print1(handle, line, col, fmt, a0)

#endif
      
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
