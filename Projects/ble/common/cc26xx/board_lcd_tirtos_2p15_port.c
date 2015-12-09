/**
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
#include <ti/drivers/lcd/LCDDogm1286.h>

#include "board_lcd.h"
#include "Board.h"

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

// LCD parameter
static LCD_Handle lcdHandle = NULL;

#ifdef TI_DRIVERS_LCD_INCLUDED
// LCD pin table
PIN_Config LCDPinTable[] = {
    Board_3V3_EN     | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL,  // Enable 3V3 domain. Need to be high for LCD to work.
    PIN_TERMINATE                                                           // Terminate list
};

// LCD pin state
PIN_State LCDPinState;

// LCD pin handle
PIN_Handle hLCDPins;
#endif //TI_DRIVERS_LCD_INCLUDED

Char lcdBuffer0[LCD_BYTES] = {0};

LCD_Buffer lcdBuffers[] = {
      {lcdBuffer0, LCD_BYTES, NULL},
  };
  
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Board_openLCD
 *
 * @brief   Open LCD peripheral on SRF06EB.
 *
 * @param   none
 *
 * @return  void
 */
void Board_openLCD(void)
{
#ifdef TI_DRIVERS_LCD_INCLUDED
  //Enable the 3V3 domain for the LCD
  hLCDPins = PIN_open(&LCDPinState, LCDPinTable);
    
  LCD_Params lcdParams;
  
  LCD_Params_init(&lcdParams);
  
  // Open LCD peripheral
  lcdHandle = LCD_open(&lcdBuffers[0], 1, &lcdParams);
  
  if ( lcdHandle )
  {
    LCD_bufferClear(lcdHandle, 0);
    LCD_update(lcdHandle, 0);
  }
#endif
}

/*********************************************************************
 * @fn      Board_writeString
 *
 * @brief   Write a string on the LCD display.
 *
 * @param   str - string to print
 * @param   line - line (page) to write (0-7)
 *
 * @return  void
 */
void Board_writeString(char *str, uint8_t line)
{
  if (lcdHandle != NULL)
  {
    LCD_bufferClearPage(lcdHandle, 0, (LCD_Page)line);
    LCD_bufferPrintString(lcdHandle, 0, str, 0, (LCD_Page)line);
    LCD_update(lcdHandle, 0);
  }
}

/*********************************************************************
 * @fn      Board_writeStringValue
 *
 * @brief   Write a string and value on the LCD display.
 *
 * @param   str - string to print
 * @param   value - value to print
 * @param   format - base of the value to print (2,8,16 etc)
 * @param   line - line (page) to write (0-7)
 *
 * @return  void
 */
void Board_writeStringValue(char *str, uint32_t value, uint8_t format,
                            uint8_t line)
{
  if (lcdHandle != NULL)
  {
    // Write string and 32-bit number
    LCD_writeLine(lcdHandle, 0, str, value, format, line);
  }
}

/*********************************************************************
*********************************************************************/
