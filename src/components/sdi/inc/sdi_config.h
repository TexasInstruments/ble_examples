/******************************************************************************

 @file  npi_config.h

 This file contains the Network Processor Interface (NPI),
 which abstracts the physical link between the Application
 Processor (AP) and the Network Processor (NP). The NPI
 serves as the HAL's client for the SPI and UART drivers, and
 provides API and callback services for its client.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************

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

#ifndef SDI_CONFIG_H
#define SDI_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************

#include "Board.h"

// ****************************************************************************
// defines
// ****************************************************************************

#if !defined(NPI_SPI_CONFIG)
#define NPI_SPI_CONFIG Board_SPI1
#endif

#ifdef POWER_SAVING
#  if defined(NPI_USE_SPI)
#       if (NPI_SPI_CONFIG == Board_SPI1)
#               define MRDY_PIN Board_KEY_UP
#               define SRDY_PIN Board_KEY_DOWN
#       else
#               error "ERROR: You must choose Board_SPI1 SPI module for NPI."
#       endif
#  elif defined(SDI_USE_UART)
#    define MRDY_PIN Board_BUTTON0
#    define SRDY_PIN Board_BUTTON1
#  endif
#  define SRDY_ENABLE()                   PIN_setOutputValue(hSdiHandshakePins, SRDY_PIN, 0) /* RTS low */
#  define SRDY_DISABLE()                  PIN_setOutputValue(hSdiHandshakePins, SRDY_PIN, 1) /* RTS high */
#else // ! POWER_SAVING
#  define SRDY_ENABLE()
#  define SRDY_DISABLE()
#endif

#define SDI_TL_BUF_SIZE         270
#define SDI_SPI_PAYLOAD_SIZE    255
#define SDI_SPI_HDR_LEN         4

#ifdef NPI_USE_SPI
#  if (NPI_TL_BUF_SIZE - NPI_SPI_HDR_LEN) < NPI_SPI_PAYLOAD_SIZE
#    define NPI_MAX_FRAG_SIZE       (NPI_TL_BUF_SIZE - NPI_SPI_HDR_LEN)
#  else
#    define NPI_MAX_FRAG_SIZE       NPI_SPI_PAYLOAD_SIZE
#  endif
#elif SDI_USE_UART
#  define SDI_MAX_FRAG_SIZE       SDI_TL_BUF_SIZE
#else
#  error "ERROR: SDI_USE_UART or NPI_USE_SPI must be defined."
#endif

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//*****************************************************************************
// function prototypes
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif /* NPI_CONFIG_H */
