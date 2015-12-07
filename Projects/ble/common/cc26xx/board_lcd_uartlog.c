/*
 * Filename:      board_lcd_uartlog.c
 *
 * Description:   This file implements the board_lcd interface used by several
 *                BLE SDK sample applications, but redirects the output to
 *                the TI-RTOS Log system instead of an LCD.
 *
 *                Also initializes a sample "Log to UART" plugin, so that Log
 *                statements are routed to the UART.
 *
 *                The rest of the UART-Log initialization is done in the TI-RTOS
 *                config file.
 *
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*********************************************************************
 * INCLUDES
 */
#include <Board.h>
#include <uart_logs.h>

#include <xdc/runtime/Log.h>   // For Log_warning1("Warning number #%d", 4); things
#include <xdc/runtime/Diags.h> // For Log_print0(Diags_USER1, "hello"); things.

#include <string.h>

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Board_openLCD
 *
 * @brief   Open the UART port and initialize Log callback plugin.
 *
 * @param   none
 *
 * @return  void
 */
void Board_openLCD(void)
{
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    UART_Handle hUart = UART_open(Board_UART, &uartParams);
    
    UartLog_init(hUart);
}

/*********************************************************************
 * @fn      Board_writeString
 *
 * @brief   Write a string to the Log system
 *
 * @param   str - string to print
 * @param   line - line (page) to write (0-7) - ignored for UART
 *
 * @return  void
 */
void Board_writeString(char *str, uint8_t line)
{
  if (strlen(str))
    Log_print1(Diags_USER6, "LCD: %$S", (IArg)str);
}

/*********************************************************************
 * @fn      Board_writeStringValue
 *
 * @brief   Write a string and value to the Log system
 *
 * @param   str - string to print
 * @param   value - value to print
 * @param   format - base of the value to print (10 and 16 supported for UART).
 * @param   line - line (page) to write (0-7) - ignored for UART
 *
 * @return  void
 */
void Board_writeStringValue(char *str, uint32_t value, uint8_t format,
                            uint8_t line)
{
  char *fmt;
  
  switch (format)
  {
  case 10:
    fmt = "LCD: %$S %d";
    break;
  case 16:
  default:
    fmt = "LCD: %$S %x";
    break;
  }
 
  Log_print2(Diags_USER6, fmt, (IArg)str, (IArg)value);
}

/*********************************************************************
*********************************************************************/
