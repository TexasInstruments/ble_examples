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
//
//  Content        : This file implements the packet handler functions used
//                   by the boot loader.
//
//------------------------------------------------------------------------------

#include <inc/hw_types.h>
#include <boot_loader.h>
#include <blcmds.h>

//*****************************************************************************
//
// The packet that is sent to acknowledge a received packet.
//
//*****************************************************************************
static const uint8_t g_ui8ACK[2] = { 0, COMMAND_ACK };

//*****************************************************************************
//
// The packet that is sent to not-acknowledge a received packet.
//
//*****************************************************************************
static const uint8_t g_ui8NAK[2] = { 0, COMMAND_NAK };

//*****************************************************************************
//
//! Calculates an 8 bit checksum
//!
//! \param pui8Data is a pointer to an array of 8 bit data of size ui32Size.
//! \param ui32Size is the size of the array that will run through the checksum
//! algorithm.
//!
//! This function simply calculates an 8 bit checksum on the data passed in.
//!
//! \return The function returns the calculated checksum.
//
//*****************************************************************************
static uint32_t
CheckSum(const uint8_t *pui8Data, uint32_t ui32Size)
{
    uint32_t ui32Offset, ui32CheckSum;

    //
    // Initialize the checksum to zero.
    //
    ui32CheckSum = 0;

    //
    // Add up all the bytes, do not do anything for an overflow.
    //
    for(ui32Offset = 0; ui32Offset < ui32Size; ui32Offset++)
    {
        ui32CheckSum += pui8Data[ui32Offset];
    }

    //
    // Return the calculated check sum.
    //
    return(ui32CheckSum & 0xff);
}

//*****************************************************************************
//
//! Sends an Acknowledge packet.
//!
//! This function is called to acknowledge that a packet has been received by
//! the device.
//!
//! \return None
//
//*****************************************************************************
void
AckPacket(void)
{
    //
    // ACK/NAK packets are the only ones with no size.
    //
    SendData(g_ui8ACK, 2);
}

//*****************************************************************************
//
//! Receives a data packet.
//!
//! \param pui8Data is the location to store the data that is sent to the boot
//! loader.
//! \param pui32Size is the number of bytes returned in the pui8Data buffer that
//! was provided.
//!
//! This function receives a packet of data from specified transfer function by
//! calling the global function pointer, ReceiveData(). This should have been
//! set to a known function before attempting to send or receive any packets.
//! A maximum of 256 bytes can be received.
//!
//! \returns The function returns zero to indicate success while any non-zero
//! value indicates a failure.
//
//*****************************************************************************
int
ReceivePacket(uint8_t *pui8Data, uint32_t *pui32Size)
{
    uint8_t ui8CheckSum, ui8Size;

    //
    // Wait for non-zero data before getting the first byte that holds the
    // size of the packet we are receiving.
    //
    ui8Size = 0;
    while(ui8Size == 0)
    {
        ReceiveData(&ui8Size, 1);
    }

    //
    // Subtract off the size and checksum bytes.
    //
    ui8Size -= 2;

    //
    // Receive the checksum followed by the actual data.
    //
    ReceiveData(&ui8CheckSum, 1);

    //
    // Receive the actual data in the packet.
    //
    ReceiveData(pui8Data, ui8Size);

    //
    // Send a not acknowledge if the checksum does not match, otherwise send
    // an acknowledge to the packet later.
    //
    if(CheckSum(pui8Data, ui8Size) != ui8CheckSum)
    {
        //
        // Indicate that the packet was not received correctly.
        //
        SendData(g_ui8NAK, 2);

        //
        // Packet was not received, there is no valid data in the buffer.
        //
        return(-1);
    }

    //
    // Make sure to return the number of bytes received.
    //
    *pui32Size = ui8Size;

    //
    // Packet was received successfully.
    //
    return(0);
}

//*****************************************************************************
//
//! Sends a data packet.
//!
//! \param pui8Data is the location of the data to be sent to the
//! microcontroller.
//! \param ui32Size is the number of bytes to send.
//!
//! This function sends the data provided in the pui8Data parameter in the
//! packet format used by the boot loader.  The caller only needs to specify
//! the buffer with the data that needs to be transferred.  This call addresses
//! all other packet formatting issues.
//!
//! \return The function returns zero to indicate success while any non-zero
//! value indicates a failure.
//
//*****************************************************************************
int
SendPacket(uint8_t *pui8Data, uint32_t ui32Size)
{
    uint8_t ui8Temp;

    //
    // Calculate the checksum to be sent out with the data.
    //
    ui8Temp = CheckSum(pui8Data, ui32Size);

    //
    // Need to include the size and checksum bytes in the packet.
    //
    ui32Size += 2;

    //
    // Send out the size followed by the data.
    //
    SendData((uint8_t *)&ui32Size, 1);
    SendData(&ui8Temp, 1);
    SendData(pui8Data, ui32Size - 2);

    //
    // Wait for a non zero byte.
    //
    ui8Temp = 0;
    while(ui8Temp == 0)
    {
        ReceiveData(&ui8Temp, 1);
    }

    //
    // Check if the byte was a valid ACK and return a negative value if it was
    // not an acknowledge.
    //
    if(ui8Temp != COMMAND_ACK)
    {
        return(-1);
    }

    //
    // This packet was sent and received successfully.
    //
    return(0);
}
