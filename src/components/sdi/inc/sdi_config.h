//******************************************************************************
//! \file           npi_config.h
//! \brief          This file contains the Network Processor Interface (NPI),
//!                 which abstracts the physical link between the Application
//!                 Processor (AP) and the Network Processor (NP). The NPI
//!                 serves as the HAL's client for the SPI and UART drivers, and
//!                 provides API and callback services for its client.
//
//   Revised        $Date: 2015-02-16 16:57:25 -0800 (Mon, 16 Feb 2015) $
//   Revision:      $Revision: 42655 $
//
//  Copyright 2015 Texas Instruments Incorporated. All rights reserved.
//
// IMPORTANT: Your use of this Software is limited to those specific rights
// granted under the terms of a software license agreement between the user
// who downloaded the software, his/her employer (which must be your employer)
// and Texas Instruments Incorporated (the "License").  You may not use this
// Software unless you agree to abide by the terms of the License. The License
// limits your use, and you acknowledge, that the Software may not be modified,
// copied or distributed unless used solely and exclusively in conjunction with
// a Texas Instruments radio frequency device, which is integrated into
// your product.  Other than for the foregoing purpose, you may not use,
// reproduce, copy, prepare derivative works of, modify, distribute, perform,
// display or sell this Software and/or its documentation for any purpose.
//
//  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
//  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,l
//  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
//  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
//  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
//  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
//  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
//  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
//  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
//  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
//  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
//
//  Should you have any questions regarding your right to use this Software,
//  contact Texas Instruments Incorporated at www.TI.com.
//******************************************************************************
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
#    define MRDY_PIN Board_KEY_UP
#    define SRDY_PIN Board_KEY_DOWN
#  endif
#  define SRDY_ENABLE()                   PIN_setOutputValue(hSdiHandshakePins, SRDY_PIN, 0) /* RTS low */
#  define SRDY_DISABLE()                  PIN_setOutputValue(hSdiHandshakePins, SRDY_PIN, 1) /* RTS high */
#else // ! POWER_SAVING
#  define SRDY_ENABLE()
#  define SRDY_DISABLE()
#endif

#define SDI_TL_BUF_SIZE         512 //270
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
