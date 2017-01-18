/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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

/* -----------------------------------------------------------------------------
 *  Includes
 * -----------------------------------------------------------------------------
 */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/UART.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/System.h>

#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayUart.h>

#include <string.h>

/* -----------------------------------------------------------------------------
 *  Constants and macros
 * -----------------------------------------------------------------------------
 */
#ifndef MIN
#  define MIN(n, m)    (((n) > (m)) ? (m) : (n))
#endif

#define DISPLAY_UART_ESC_INITIAL            "\x1b\x63"    /* Reset terminal */                \
                                            "\x1b[2J"     /* Clear screen   */                \
                                            "\x1b[10r"    /* Scrolling region from line 10 */ \
                                            "\x1b[11;1H"  /* Set initial cursor to line 11 */

#define DISPLAY_UART_ESC_MOVEPOS_FMT        "\x1b\x37"    /* Save cursor position */ \
                                            "\x1b[10r"    /* Retransmit scroll    */ \
                                            "\x1b[%d;%dH" /* Move cursor fmt str  */

#define DISPLAY_UART_ESC_RESTOREPOS         "\x1b\x38"    /* Restore saved cursor pos */

#define DISPLAY_UART_ESC_CLEAR_SCREEN       "\x1b[2J"     /* Clear screen       */
#define DISPLAY_UART_ESC_CLEAR_CUR_LEFT     "\x1b[1K"     /* Clear cursor left  */
#define DISPLAY_UART_ESC_CLEAR_CUR_RIGHT    "\x1b[0K"     /* Clear cursor right */
#define DISPLAY_UART_ESC_CLEAR_BOTH         "\x1b[2K"     /* Clear line         */
#define DISPLAY_UART_ESC_CLEARSEQ_LEN       4

/* -----------------------------------------------------------------------------
 *   Type definitions
 * -----------------------------------------------------------------------------
 */


/* -----------------------------------------------------------------------------
 *                           Local variables
 * -----------------------------------------------------------------------------
 */
/* Display function table for UART implementation */
const Display_FxnTable DisplayUart_fxnTable = {
    DisplayUart_open,
    DisplayUart_clear,
    DisplayUart_clearLines,
    DisplayUart_put5,
    DisplayUart_close,
    DisplayUart_control,
    DisplayUart_getType,
};

/* -----------------------------------------------------------------------------
 *                                          Functions
 * -----------------------------------------------------------------------------
 */
/*!
 * @fn          DisplayUart_open
 *
 * @brief       Initialize the UART transport
 *
 * @descr       Opens the UART index specified in the HWAttrs, and creates a
 *              mutex semaphore
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       params - display parameters
 *
 * @return      Pointer to Display_Config struct
 */
Display_Handle DisplayUart_open(Display_Handle hDisplay,
                                Display_Params *params)
{
    DisplayUart_HWAttrs *hwAttrs = (DisplayUart_HWAttrs *)hDisplay->hwAttrs;
    DisplayUart_Object  *object  = (DisplayUart_Object  *)hDisplay->object;

    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.baudRate  = hwAttrs->baudRate;
    uartParams.writeMode = UART_MODE_BLOCKING;

    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&object->mutex, 1, &semParams);

    switch (params->lineClearMode)
    {
        case DISPLAY_CLEAR_BOTH:  object->lineClearSeq = DISPLAY_UART_ESC_CLEAR_BOTH; break;
        case DISPLAY_CLEAR_LEFT:  object->lineClearSeq = DISPLAY_UART_ESC_CLEAR_CUR_LEFT; break;
        case DISPLAY_CLEAR_RIGHT: object->lineClearSeq = DISPLAY_UART_ESC_CLEAR_CUR_RIGHT; break;
        default: /* fall-through */
        case DISPLAY_CLEAR_NONE: object->lineClearSeq = NULL; break;
    }

    object->hUart = UART_open(hwAttrs->uartIdx, &uartParams);
    if (NULL == object->hUart)
    {
        Log_print0(Diags_USER1, "DisplayUart.c: Couldn't open UART");
        return NULL;
    }

    /* Send VT100 initial config to terminal */
    UART_write(object->hUart, DISPLAY_UART_ESC_INITIAL, sizeof DISPLAY_UART_ESC_INITIAL - 1);

    return hDisplay;
}

/*!
 * @fn          DisplayUart_clear
 *
 * @brief       Does nothing, as output is stateless.
 *
 * @param       hDisplay - pointer to Display_Config struct
 *
 * @return      void
 */
void DisplayUart_clear(Display_Handle hDisplay)
{
    DisplayUart_Object  *object  = (DisplayUart_Object  *)hDisplay->object;
    DisplayUart_HWAttrs *hwAttrs = (DisplayUart_HWAttrs *)hDisplay->hwAttrs;

    if (Semaphore_pend((Semaphore_Handle) & object->mutex, hwAttrs->mutexTimeout))
    {
        UART_write(object->hUart, DISPLAY_UART_ESC_CLEAR_SCREEN, 4);
        Semaphore_post((Semaphore_Handle) & object->mutex);
    }
}


/*!
 * @fn          DisplayUart_clearLines
 *
 * @brief       Does nothing, as output is stateless.
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       lineFrom - line index (0 .. )
 * @param       lineTo - line index (0 .. )
 *
 * @return      void
 */
void DisplayUart_clearLines(Display_Handle hDisplay,
                            uint8_t lineFrom, uint8_t lineTo)
{
    DisplayUart_Object  *object  = (DisplayUart_Object  *)hDisplay->object;
    DisplayUart_HWAttrs *hwAttrs = (DisplayUart_HWAttrs *)hDisplay->hwAttrs;

    uint32_t      strSize = 0;
    uint32_t      curLine = 0;
    const uint8_t uartClearLineMoveDown[] = "\x1b[2K\x1b\x45";

    if (lineTo <= lineFrom)
    {
        lineTo = lineFrom;
    }

    if (Semaphore_pend((Semaphore_Handle) & object->mutex, hwAttrs->mutexTimeout))
    {
        strSize += System_snprintf(hwAttrs->strBuf, hwAttrs->strBufLen,
                                   DISPLAY_UART_ESC_MOVEPOS_FMT, lineFrom + 1, 0);

        for (curLine = lineFrom + 1;
        		curLine < lineTo + 2 &&
                	hwAttrs->strBufLen - strSize > sizeof DISPLAY_UART_ESC_RESTOREPOS - 1;
        		curLine++)
        {
            memcpy(hwAttrs->strBuf + strSize, uartClearLineMoveDown, sizeof uartClearLineMoveDown - 1);
            strSize += sizeof uartClearLineMoveDown - 1;
        }

        memcpy(hwAttrs->strBuf + strSize, DISPLAY_UART_ESC_RESTOREPOS, sizeof DISPLAY_UART_ESC_RESTOREPOS - 1);
        strSize += sizeof DISPLAY_UART_ESC_RESTOREPOS - 1;

        UART_write(object->hUart, hwAttrs->strBuf, strSize);
        Semaphore_post((Semaphore_Handle) & object->mutex);
    }
}


/*!
 * @fn          DisplayUart_put5
 *
 * @brief       Write a text string to UART with return and newline.
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       line - line index (0..)
 * @param       column - column index (0..)
 * @param       fmt - format string
 * @param       aN - optional format arguments
 *
 * @return      void
 */
void DisplayUart_put5(Display_Handle hDisplay, uint8_t line,
                      uint8_t column, uintptr_t fmt, uintptr_t a0,
                      uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4)
{
    DisplayUart_Object  *object  = (DisplayUart_Object  *)hDisplay->object;
    DisplayUart_HWAttrs *hwAttrs = (DisplayUart_HWAttrs *)hDisplay->hwAttrs;

    uint32_t       strSize = 0;

    char          *strBuf = hwAttrs->strBuf;
    const uint16_t bufLen = hwAttrs->strBufLen;

    if (Semaphore_pend((Semaphore_Handle) & object->mutex, hwAttrs->mutexTimeout))
    {
        if (line != DisplayUart_SCROLLING)
        {
            /* Add cursor movement escape sequence */
            strSize += System_snprintf(strBuf + strSize, bufLen - strSize - 2,
                                       DISPLAY_UART_ESC_MOVEPOS_FMT, line + 1, column + 1);

            /* Add line clearing escape sequence */
            if (object->lineClearSeq)
            {
                memcpy(strBuf + strSize, object->lineClearSeq, DISPLAY_UART_ESC_CLEARSEQ_LEN);
                strSize += DISPLAY_UART_ESC_CLEARSEQ_LEN;
            }
        }

        strSize += System_snprintf(strBuf + strSize, bufLen - strSize - 2, (xdc_CString)fmt, a0, a1, a2, a3, a4);

        if (line != DisplayUart_SCROLLING)
        {
            memcpy(strBuf + strSize, DISPLAY_UART_ESC_RESTOREPOS, sizeof DISPLAY_UART_ESC_RESTOREPOS - 1);
            strSize += 2;
        }
        else
        {
            strBuf[strSize++] = '\r';
            strBuf[strSize++] = '\n';
        }

        UART_write(object->hUart, strBuf, strSize);
        Semaphore_post((Semaphore_Handle) & object->mutex);
    }
}


/*!
 * @fn          DisplayUart_close
 *
 * @brief       Closes the UART handle
 *
 * @param       hDisplay - pointer to Display_Config struct
 *
 * @return      void
 */
void DisplayUart_close(Display_Handle hDisplay)
{
    DisplayUart_Object *object = (DisplayUart_Object  *)hDisplay->object;

    // Not sure what happens if someone is writing
    UART_close(object->hUart);
    object->hUart = NULL;

    // Not sure what happens if someone is pending
    Semaphore_destruct(&object->mutex);
}

/*!
 * @fn          DisplayUart_control
 *
 * @brief       Function for setting control parameters of the Display driver
 *              after it has been opened.
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       cmd - command to execute
 * @param       arg - argument to the command
 *
 * @return      ::DISPLAY_STATUS_UNDEFINEDCMD because no commands are supported
 */
int DisplayUart_control(Display_Handle hDisplay, unsigned int cmd, void *arg)
{
    return DISPLAY_STATUS_UNDEFINEDCMD;
}

/*!
 * @fn          DisplayUart_getType
 *
 * @brief       Returns type of transport
 *
 * @return      Display type UART
 */
unsigned int DisplayUart_getType(void)
{
    return Display_Type_UART;
}
