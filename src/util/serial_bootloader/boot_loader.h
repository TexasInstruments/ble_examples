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
//  Content        : Definitions and prototypes used by the boot loader.
//
//------------------------------------------------------------------------------
#ifndef __BOOT_LOADER_H__
#define __BOOT_LOADER_H__

//*****************************************************************************
//
// Definitions for reading CCFG configuration parameter values
//
//*****************************************************************************
#define CCFG_OFFSET            0x3000


//*****************************************************************************
//
// The communcation interface that was detected to be in use.
//
//*****************************************************************************
#define UART_MODE               1
#define SSP_MODE                2

//*****************************************************************************
//
// These function pointers indicate the functions to transmit and receive over
// the selected interface.
//
//*****************************************************************************
extern void (*SendData)(const uint8_t *pui8Data, uint32_t ui32Size);
extern void (*ReceiveData)(uint8_t *pui8Data, uint32_t ui32Size);
extern void (*Flush)(void);

//*****************************************************************************
//
// Function used by GPIO interrupt when using UART bootloader
//
//*****************************************************************************
extern void GPIOIntHandler(void);

//*****************************************************************************
//
// UART Transport APIs
//
//*****************************************************************************
extern void UARTConfig(int iProcRatio);
extern void UARTSend(const uint8_t *pui8Data, uint32_t ui32Size);
extern void UARTFlush(void);
extern void UARTReceive(uint8_t *pui8Data, uint32_t ui32Size);
extern int  UARTAutoBaud(uint32_t *pui32Ratio);

//*****************************************************************************
//
// SSP Transport APIs
//
//*****************************************************************************
extern void SSPConfig(void);
extern void SSPSend(const uint8_t *pui8Data, uint32_t ui32Size);
extern void SSPFlush(void);
extern void SSPReceive(uint8_t *pui8Data, uint32_t ui32Size);
extern void SSPDisableInterface(void);

//*****************************************************************************
//
// Packet Handling APIs
//
//*****************************************************************************
extern int ReceivePacket(uint8_t *pui8Data, uint32_t *pui32Size);
extern int SendPacket(uint8_t *pui8Data, uint32_t ui32Size);
extern void AckPacket(void);

//*****************************************************************************
//
// Bootloader APIs.
//
//*****************************************************************************
extern void PickInterface(void);
extern void Update(void);
extern int  bootloaderOpened(void);
#endif // __BOOT_LOADER_H__
