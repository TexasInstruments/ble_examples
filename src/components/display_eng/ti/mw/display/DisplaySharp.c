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
 * ------------------------------------------------------------------------------
 */
// TI RTOS drivers
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>

#include <ti/mw/grlib/grlib.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplaySharp.h>
#include <ti/mw/lcd/SharpGrLib.h>

/* -----------------------------------------------------------------------------
 *  Constants and macros
 * ------------------------------------------------------------------------------
 */
// Timeout of semaphore that controls exclusive to the LCD (infinite)
#define ACCESS_TIMEOUT    BIOS_WAIT_FOREVER

/* -----------------------------------------------------------------------------
 *   Type definitions
 * ------------------------------------------------------------------------------
 */


/* -----------------------------------------------------------------------------
 *                           Local variables
 * ------------------------------------------------------------------------------
 */
/* Display function table for sharp implementation */
const Display_FxnTable DisplaySharp_fxnTable = {
    DisplaySharp_open,
    DisplaySharp_clear,
    DisplaySharp_clearLines,
    DisplaySharp_put5,
    DisplaySharp_close,
    DisplaySharp_control,
    DisplaySharp_getType,
};

/* -----------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------
 */
/*!
 * @fn          DisplaySharp_open
 *
 * @brief       Initialize the LCD
 *
 * @descr       Initializes the pins used by the LCD, creates resource access
 *              protection semaphore, turns on the LCD device, initializes the
 *              frame buffer, initializes to white background/dark foreground,
 *              and finally clears the object->displayColor.
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       params - display parameters
 *
 * @return      Pointer to Display_Config struct
 */
Display_Handle DisplaySharp_open(Display_Handle hDisplay,
                                 Display_Params *params)
{
    DisplaySharp_HWAttrs *hwAttrs = (DisplaySharp_HWAttrs *)hDisplay->hwAttrs;
    DisplaySharp_Object  *object  = (DisplaySharp_Object  *)hDisplay->object;

    PIN_Config pinTable[4 + 1];

    object->lineClearMode = params->lineClearMode;

    uint32_t i = 0;
    if (hwAttrs->csPin != PIN_TERMINATE)
    {
        pinTable[i++] = hwAttrs->csPin | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX;
    }
    if (hwAttrs->extcominPin != PIN_TERMINATE)
    {
        pinTable[i++] = hwAttrs->extcominPin | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX;
    }
    if (hwAttrs->powerPin != PIN_TERMINATE)
    {
        pinTable[i++] = hwAttrs->powerPin | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX;
    }
    if (hwAttrs->enablePin != PIN_TERMINATE)
    {
        pinTable[i++] = hwAttrs->enablePin | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX;
    }
    pinTable[i++] = PIN_TERMINATE;

    object->hPins = PIN_open(&object->pinState, pinTable);
    if (object->hPins == NULL)
    {
        Log_error0("Couldn't open pins for Sharp96x96");
        return NULL;
    }

    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 4000000;

    object->hSpi = SPI_open(hwAttrs->spiIndex, &spiParams);

    if (object->hSpi == NULL)
    {
        Log_error0("Couldn't open SPI driver for Sharp96x96");
        PIN_close(object->hPins);
        object->hPins = NULL;
        return NULL;
    }

    // Init colors
    object->displayColor.bg = ClrBlack;
    object->displayColor.fg = ClrWhite;

    // Exclusive access
    Semaphore_Params semParams;

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&object->semLCD, 1, &semParams);

    // Grab LCD
    Semaphore_pend((Semaphore_Handle) & object->semLCD, ACCESS_TIMEOUT);

    // Initialize the GrLib back-end transport
    SharpGrLib_init(object->hSpi, object->hPins, hwAttrs->csPin);

    object->g_sDisplay.lSize         = sizeof(tDisplay);
    object->g_sDisplay.pFxns         = &g_sharpFxns;
    object->g_sDisplay.pvDisplayData = object->displayBuffer;
    object->g_sDisplay.usHeight      = hwAttrs->pixelHeight;
    object->g_sDisplay.usWidth       = hwAttrs->pixelWidth;
    object->g_sDisplay.pvDisplayData = hwAttrs->displayBuf;

    // Graphics library init
    GrContextInit(&object->g_sContext, &object->g_sDisplay, &g_sharpFxns);

    // Graphics properties
    GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
    GrContextBackgroundSet(&object->g_sContext, object->displayColor.bg);
    GrContextFontSet(&object->g_sContext, &g_sFontFixed6x8);

    // Clear display
    GrClearDisplay(&object->g_sContext);
    GrFlush(&object->g_sContext);

    // Release LCD
    Semaphore_post((Semaphore_Handle) & object->semLCD);

    return hDisplay;
}


/*!
 * @fn          DisplaySharp_clear
 *
 * @brief       Clears the display
 *
 * @param       hDisplay - pointer to Display_Config struct
 *
 * @return      void
 */
void DisplaySharp_clear(Display_Handle hDisplay)
{
    DisplaySharp_Object *object = (DisplaySharp_Object  *)hDisplay->object;

    if (object->hPins == NULL)
    {
        return;
    }

    // Grab LCD
    if (Semaphore_pend((Semaphore_Handle) & object->semLCD, ACCESS_TIMEOUT))
    {
        GrClearDisplay(&object->g_sContext);
        GrFlush(&object->g_sContext);

        // Release LCD
        Semaphore_post((Semaphore_Handle) & object->semLCD);
    }
}


/*!
 * @fn          DisplaySharp_clearLines
 *
 * @brief       Clears lines lineFrom-lineTo of the display, inclusive
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       lineFrom - line index (0 .. )
 * @param       lineTo - line index (0 .. )
 *
 * @return      void
 */
void DisplaySharp_clearLines(Display_Handle hDisplay,
                             uint8_t lineFrom, uint8_t lineTo)
{
    DisplaySharp_Object *object = (DisplaySharp_Object  *)hDisplay->object;

    if (lineTo <= lineFrom)
    {
        lineTo = lineFrom;
    }

    tRectangle rect = {
        .sXMin =                                                     0,
        .sXMax = object->g_sContext.sClipRegion.sXMax,
        .sYMin = lineFrom * object->g_sContext.pFont->ucHeight,
        .sYMax = (lineTo + 1) * object->g_sContext.pFont->ucHeight - 1,
    };

    GrContextForegroundSet(&object->g_sContext, object->displayColor.bg);
    GrRectFill(&object->g_sContext, &rect);
    GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
    GrFlush(&object->g_sContext);
}


/*!
 * @fn          DisplaySharp_put5
 *
 * @brief       Write a text string to a specific line/column of the display
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       line - line index (0..)
 * @param       column - column index (0..)
 * @param       fmt - format string
 * @param       aN - optional format arguments
 *
 * @return      void
 */
void DisplaySharp_put5(Display_Handle hDisplay, uint8_t line,
                       uint8_t column, uintptr_t fmt, uintptr_t a0,
                       uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4)
{
    DisplaySharp_Object *object = (DisplaySharp_Object  *)hDisplay->object;

    uint8_t xp, yp, clearStartX, clearEndX;

    char    dispStr[23];

    if (object->hPins == NULL)
    {
        return;
    }

    // Grab LCD
    if (Semaphore_pend((Semaphore_Handle) & object->semLCD, ACCESS_TIMEOUT))
    {
        xp          = column * object->g_sContext.pFont->ucMaxWidth + 1;
        yp          = line * object->g_sContext.pFont->ucHeight + 0;
        clearStartX = clearEndX = xp;

        switch (object->lineClearMode)
        {
        case DISPLAY_CLEAR_LEFT:
            clearStartX = 0;
            break;
        case DISPLAY_CLEAR_RIGHT:
            clearEndX = object->g_sContext.sClipRegion.sXMax;
            break;
        case DISPLAY_CLEAR_BOTH:
            clearStartX = 0;
            clearEndX   = object->g_sContext.sClipRegion.sXMax;
            break;
        case DISPLAY_CLEAR_NONE:
        default:
            break;
        }

        if (clearStartX != clearEndX)
        {
            tRectangle rect = {
                .sXMin = clearStartX,
                .sXMax = clearEndX,
                .sYMin = yp,
                .sYMax = yp + object->g_sContext.pFont->ucHeight - 1,
            };

            GrContextForegroundSet(&object->g_sContext, object->displayColor.bg);
            GrRectFill(&object->g_sContext, &rect);
            GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
        }

        System_snprintf(dispStr, sizeof(dispStr), (xdc_CString)fmt, a0, a1, a2, a3, a4);

        // Draw a text on the display
        GrStringDraw(&object->g_sContext,
                     dispStr,
                     AUTO_STRING_LENGTH,
                     xp,
                     yp,
                     OPAQUE_TEXT);

        GrFlush(&object->g_sContext);

        // Release LCD
        Semaphore_post((Semaphore_Handle) & object->semLCD);
    }
}


/*!
 * @fn          DisplaySharp_close
 *
 * @brief       Turns of the display and releases the LCD control pins
 *
 * @param       hDisplay - pointer to Display_Config struct
 *
 * @return      void
 */
void DisplaySharp_close(Display_Handle hDisplay)
{
    DisplaySharp_HWAttrs *hwAttrs = (DisplaySharp_HWAttrs *)hDisplay->hwAttrs;
    DisplaySharp_Object  *object  = (DisplaySharp_Object  *)hDisplay->object;

    if (object->hPins == NULL)
    {
        return;
    }

    // Grab LCD
    if (Semaphore_pend((Semaphore_Handle) & object->semLCD, ACCESS_TIMEOUT))
    {
        // Turn off the display
        PIN_setOutputValue(object->hPins, hwAttrs->enablePin, 0);

        // Release resources
        PIN_close(object->hPins);
        object->hPins = NULL;

        SPI_close(object->hSpi);
        object->hSpi = NULL;

        // Deconfigure GrLib back-end
        SharpGrLib_init(NULL, NULL, PIN_UNASSIGNED);

        // Release LCD
        Semaphore_post((Semaphore_Handle) & object->semLCD);
    }
}

/*!
 * @fn          DisplaySharp_control
 *
 * @brief       Function for setting control parameters of the Display driver
 *              after it has been opened.
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       cmd - command to execute, supported commands are:
 *              | Command                        | Description             |
 *              |------------------------------- |-------------------------|
 *              | ::DISPLAY_CMD_TRANSPORT_CLOSE  | Close SPI but leave control pins |
 *              | ::DISPLAY_CMD_TRANSPORT_OPEN   | Re-open SPI driver      |
 * @param       arg - argument to the command
 *
 * @return      ::DISPLAY_STATUS_SUCCESS if success, or error code if error.
 */
int DisplaySharp_control(Display_Handle hDisplay, unsigned int cmd, void *arg)
{
    DisplaySharp_HWAttrs *hwAttrs = (DisplaySharp_HWAttrs *)hDisplay->hwAttrs;
    DisplaySharp_Object  *object  = (DisplaySharp_Object  *)hDisplay->object;

    /* Initialize return value */
    int ret = DISPLAY_STATUS_ERROR;

    /* Perform command */
    switch(cmd)
    {
        case DISPLAY_CMD_TRANSPORT_CLOSE:
            // Grab LCD
            if (Semaphore_pend((Semaphore_Handle) & object->semLCD, ACCESS_TIMEOUT))
            {
                if (object->hSpi)
                {
                    // Close SPI and tell back-end there is no SPI
                    SPI_close(object->hSpi);
                    SharpGrLib_init(NULL, object->hPins, hwAttrs->csPin);
                    object->hSpi = NULL;
                    ret = DISPLAY_STATUS_SUCCESS;
                }
                // Release LCD
                Semaphore_post((Semaphore_Handle) & object->semLCD);
            }
            break;

        case DISPLAY_CMD_TRANSPORT_OPEN:
            // Grab LCD
            if (Semaphore_pend((Semaphore_Handle) & object->semLCD, ACCESS_TIMEOUT))
            {
                if (NULL == object->hSpi)
                {
                    // Re-open SPI and re-init back-end
                    SPI_Params spiParams;
                    SPI_Params_init(&spiParams);
                    spiParams.bitRate = 4000000;
                    object->hSpi = SPI_open(hwAttrs->spiIndex, &spiParams);
                    SharpGrLib_init(object->hSpi, object->hPins, hwAttrs->csPin);
                    ret = DISPLAY_STATUS_SUCCESS;
                }
                // Release LCD
                Semaphore_post((Semaphore_Handle) & object->semLCD);
            }
            break;

        case DISPLAYSHARP_CMD_SET_COLORS:
            // Grab LCD
            if (Semaphore_pend((Semaphore_Handle) & object->semLCD, ACCESS_TIMEOUT))
            {
                object->displayColor = *(DisplaySharpColor_t *)arg;

                GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
                GrContextBackgroundSet(&object->g_sContext, object->displayColor.bg);

                // Release LCD
                Semaphore_post((Semaphore_Handle) & object->semLCD);

                // Return success
                ret = DISPLAY_STATUS_SUCCESS;
            }
            break;

        default:
            /* The command is not defined */
            ret = SPI_STATUS_UNDEFINEDCMD;
            break;
    }

    return ret;
}

/*!
 * @fn          DisplaySharp_getType
 *
 * @brief       Returns type of transport
 *
 * @return      Display type define LCD
 */
unsigned int DisplaySharp_getType(void)
{
    return Display_Type_LCD | Display_Type_GRLIB;
}
