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
//
//  Content        : This file contains the function used to transfer data via
//                   the SSP.
//
//------------------------------------------------------------------------------
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ssi.h>
#include <boot_loader.h>


//*****************************************************************************
//
// This is the number of bits per transfer for SSP. This is a constant and
// cannot be changed without corresponding code changes.
//
//*****************************************************************************
#define DATA_BITS_SSP           8

//*****************************************************************************
//
//! Sends data via the SSP port in slave mode.
//!
//! \param pui8Data is the location of the data to send through the SSP port.
//! \param ui32Size is the number of bytes of data to send.
//!
//! This function sends data through the SSP port in slave mode. This function
//! will not return until all bytes are sent.
//!
//! \return None
//
//*****************************************************************************
void
SSPSend(const uint8_t *pui8Data, uint32_t ui32Size)
{
    //
    // Send the requested number of bytes over the SSP Port.
    //
    while(ui32Size--)
    {
        //
        // Wait until there is space.
        //
        while(!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TNF))
        {
        }

        //
        // Write the data to the SSP.
        //
        HWREG(SSI0_BASE + SSI_O_DR) = *pui8Data;
        pui8Data++;
    }

    //
    // Empty the receive FIFO.
    //
    while(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE)
    {
        // Data to be read. Read data.
        HWREG(SSI0_BASE + SSI_O_DR);
    }
}

//*****************************************************************************
//
//! Waits until all data has been transmitted by the SSP port.
//!
//! This function waits until all data written to the SSP port has been read by
//! the master.
//!
//! \return None.
//
//*****************************************************************************
void
SSPFlush(void)
{
    //
    // Wait until the transmit FIFO is empty.
    //
    while(!HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TFE)
    {
    }

    //
    // Wait until the interface is not busy.
    //
    while(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_BSY)
    {
    }
}

//*****************************************************************************
//
//! Receives data from the SSP port in slave mode.
//!
//! \param pui8Data is the location to store the data received from the SSP
//! port.
//! \param ui32Size is the number of bytes of data to receive.
//!
//! This function receives data from the SSP port in slave mode. The function
//! will not return until ui32Size number of bytes have been received.
//!
//! \return None.
//
//*****************************************************************************
void
SSPReceive(uint8_t *pui8Data, uint32_t ui32Size)
{
    //
    // Ensure that we are sending out zeros so that we don't confuse the host.
    //
    HWREG(SSI0_BASE + SSI_O_DR) = 0;

    //
    // Wait for the requested number of bytes.
    //
    while(ui32Size--)
    {
        //
        // Wait until there is data in the FIFO.
        //
        while(!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE))
        {
        }

        //
        // Read the next byte from the FIFO.
        //
        *pui8Data++ = HWREG(SSI0_BASE + SSI_O_DR);
        HWREG(SSI0_BASE + SSI_O_DR) = 0;
    }
}

//*****************************************************************************
//
//! Configures the SSP port for slave operation.
//!
//! This function configures the SSP port as a slave and prepares it to
//! transmit/receive data from a SSP master device.
//! This function sets the SSP protocol to Motorola with default clock high and
//! data valid on the rising edge.
//!
//! \return None.
//
//*****************************************************************************
void
SSPConfig(void)
{
    //
    // Set the SSP protocol to Motorola with default clock high and data
    // valid on the rising edge.
    //
    HWREG(SSI0_BASE + SSI_O_CR0) = (SSI_CR0_SPH | SSI_CR0_SPO |
                                    (DATA_BITS_SSP - 1));

    //
    // Set the SSP interface in slave mode.
    //
    HWREG(SSI0_BASE + SSI_O_CR1) = SSI_CR1_MS;

    //
    // Enable the SSP interface.
    //
    HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_SSE;
}

//*****************************************************************************
//
//! Disables the SSP interface.
//!
//! This function disables the SSP.
//!
//! \return None
//
//*****************************************************************************
void
SSPDisableInterface(void)
{
    HWREG(SSI0_BASE + SSI_O_CR1) = 0;
}
