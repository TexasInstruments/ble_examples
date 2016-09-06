//------------------------------------------------------------------------------
//  Copyright (C) 2011-2016 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//  Content        : This file holds the implementation of the automatic baud
//                   rate detection code.
//
//------------------------------------------------------------------------------

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_ssi.h>
#include <inc/hw_ioc.h>
#include <driverlib/ioc.h>
#include <inc/hw_nvic.h>
#include <inc/hw_prcm.h>
#include <inc/hw_gpio.h>
#include <inc/hw_fcfg1.h>
#include <boot_loader.h>

//*****************************************************************************
//
// This define holds the maximum number of edges that are sent as a result of
// a pattern of 2 bytes.
// Each byte includes: 1 start bit + 8 data bits + 1 stop bit = 10 bits.
// Each Tick Buffer location holds the no of systicks counted between two
// consequitive edges which defines the bit-length.
//
//*****************************************************************************
#define TICK_BUFFER_SIZE        20

//*****************************************************************************
//
// This define holds the multiplier for the pulse detection algorithm.  The
// value is used to generate a fractional difference detection of
// 1 / PULSE_DETECTION_MULT.
//
//*****************************************************************************
#define PULSE_DETECTION_MULT    3

//*****************************************************************************
//
// This define holds the minimum number of edges to successfully sync to a
// pattern of 2 bytes.
//
//*****************************************************************************
#define MIN_EDGE_COUNT          18

//*****************************************************************************
//
// This global holds the number of edges that have been stored in the global
// buffer g_lTickBuffer.
//
//*****************************************************************************
static volatile uint32_t g_ui32TickIndex;

//*****************************************************************************
//
// This global buffer is used to hold the timing of each edge for baud rate
// detection.
//
//*****************************************************************************
static volatile int32_t g_i32TickBuffer[TICK_BUFFER_SIZE];

//*****************************************************************************
//
//! Interrupt handler for edge detect interrupt on IO-pin.
//!
//! This interrupt handler takes care of saving the timing of the pulses
//! received on the UART_RX pin, used to calculate what to set the dividers
//! in the UART to.
//!
//! \returns None
//
//*****************************************************************************
void
GPIOIntHandler(void)
{
    int32_t i32Temp;

    // Clear the IOC edge detect interrupt status. Since the GPIO enabled for
    // edge detect is configurable all events are cleared to be sure the
    // correct one is hit. It is only the boot loader that uses the GPIOs at
    // this point.
    HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) = 0xFFFFFFFF;

    // While we still have space in our buffer, store the current system tick
    // count and return from interrupt.
    if(g_ui32TickIndex < TICK_BUFFER_SIZE)
    {
        i32Temp = HWREG(NVIC_ST_CURRENT);
        g_i32TickBuffer[g_ui32TickIndex++] = i32Temp;
    }
}

//*****************************************************************************
//
//! Implements the auto-baud feature for the boot loader.
//!
//! \param pui32Ratio is the ratio of the processor's crystal frequency to the
//! baud rate being used by the UART port for communications. See UARTConfig()
//! for a description of this value.
//!
//! This function attempts to synchronize to the updater program that is trying
//! to communicate with the boot loader. The UART port is monitored for edges
//! using interrupts. Once enough edges are detected, the boot loader
//! determines the ratio of baud rate and crystal frequency needed to program
//! the UART.
//!
//! Assumption: Sys clock and UART clock must run at same freq.
//! Otherwise value of pui32Ratio must be divided with the relation between
//! the clocks.
//!
//! \return Returns a value of UART_MODE indicates that this call successfully
//! synchronized with the other device communicating over the UART.
//! A value of 0 indicates that this function did not successfully synchronize
//! with the other UART device.
//! A value of SSP_MODE indicates that data have been received on the SSP port.
//
//*****************************************************************************
int
UARTAutoBaud(uint32_t *pui32Ratio)
{
    volatile int32_t i32Total;
    int32_t i32Pulse;
    int32_t i32ValidPulses;
    int32_t i32Temp;
    int32_t i32LastTotal;
    uint32_t ui32RegOffset;
    uint32_t ui32Ssi0Present;

    // Reset the counters that control the pulse detection.
    i32ValidPulses  = 0;
    i32Total        = 0;
    g_ui32TickIndex = 0;

    // Check if SSI0 is present on die
    ui32Ssi0Present = 1;

    // Enable for edge interrupt on both edges on GPIO-pin to be used for UART_RX
    HWREG(IOC_BASE + IOC_O_IOCFG0 + (PIN_UART_RXD * 4)) = IOC_IOCFG0_IE   |
            IOC_IOCFG0_EDGE_IRQ_EN  |
            IOC_IOCFG0_EDGE_DET_M |
            IOC_IOPULL_UP;

    // Clear event status register
    HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) = 0xFFFFFFFF;

    // Enable the AON edge detect interrupt
    HWREG(NVIC_EN0) = NVIC_EN0_INT0;

    // Wait for MIN_EDGE_COUNT to pass to collect enough edges.
    while(g_ui32TickIndex < MIN_EDGE_COUNT)
    {
        if(ui32Ssi0Present)
        {
            // See if data has been received on the SSP port by
            // checking if the 'SSP Receive FIFO Not Empty'-bit has been set
            if((HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE))
            {
                // Disable the AON edge detect interrupt.
                HWREG(NVIC_DIS0) = NVIC_DIS0_INT0;

                // Disable the edge interrupt on GPIO-pin used for UART_RX
                // by configuring pad back to reset value.
                HWREG(IOC_BASE + IOC_O_IOCFG0 + ui32RegOffset) = IOC_NO_IOPULL;

                // Return, selecting SSI mode.
                return(SSP_MODE);
            }
        }
    }

    // Disable the AON edge detect interrupt.
    HWREG(NVIC_DIS0) = NVIC_DIS0_INT0;

    // Disable the edge interrupt on GPIO-pin used for UART_RX
    HWREG(IOC_BASE + IOC_O_IOCFG0 + ui32RegOffset) &= ~(IOC_IOCFG0_EDGE_IRQ_EN |
            IOC_IOCFG0_EDGE_DET_M);

    // Calculate the pulse width from the array of tick times.
    for(i32Pulse = 0; i32Pulse < (g_ui32TickIndex - 1); i32Pulse++)
    {
        i32Temp = g_i32TickBuffer[i32Pulse];
        if(i32Temp < g_i32TickBuffer[i32Pulse + 1])
        {
            i32Temp -= g_i32TickBuffer[i32Pulse + 1] - 0x1000000;
        }
        else
        {
            i32Temp -= g_i32TickBuffer[i32Pulse + 1];
        }
        g_i32TickBuffer[i32Pulse] = i32Temp;
    }

    // This loops handles checking for consecutive pulses that have pulse
    // widths that are within an acceptable margin.
    for(i32Pulse = 0; i32Pulse < (g_ui32TickIndex - 1); i32Pulse++)
    {
        // Calculate the absolute difference between two consecutive pulses.
        i32Temp = g_i32TickBuffer[i32Pulse];
        i32Temp -= g_i32TickBuffer[i32Pulse + 1];
        if(i32Temp < 0)
        {
            i32Temp *= -1;
        }

        // This pulse detection code uses the following algorithm:
        // If the following is true then we have consecutive acceptable pulses
        // abs(Pulse[n] - Pulse[n + 1]) < (Pulse[n + 1] / PULSE_DETECTION_MULT)
        // or
        // (PULSE_DETECTION_MULT * abs(Pulse[n] - Pulse[n + 1])) < Pulse[n + 1]
        if((i32Temp * PULSE_DETECTION_MULT) < g_i32TickBuffer[i32Pulse + 1])
        {
            i32LastTotal = i32Total;
            i32Total = i32LastTotal + g_i32TickBuffer[i32Pulse];
            i32ValidPulses++;
        }
        else
        {
            i32ValidPulses = 0;
            i32Total = 0;
        }

        // Once we have 7 pulses calculate the ratio needed to program the UART.
        if(i32ValidPulses == 7)
        {
            // Add in the last pulse and calculate the ratio.
            i32LastTotal = i32Total;
            i32Total = i32LastTotal + g_i32TickBuffer[i32Pulse];
            *pui32Ratio = i32Total >> 1;

            // Wait for at least 2 UART clocks since we only wait for 18 of 20
            // that are coming from the host.  If we don't wait, we can turn
            // on the UART while the last two pulses come down.
            while(i32Total--)
            {
            }

            // Indicate a successful auto baud operation.
            return(UART_MODE);
        }
    }

    // Automatic baud rate detection failed.
    return(0);
}
