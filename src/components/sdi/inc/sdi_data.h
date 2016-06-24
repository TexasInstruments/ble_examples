/******************************************************************************

 @file  npi_data.h

 SDI Data structures

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

#ifndef SDIDATA_H
#define SDIDATA_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************

#include <stdint.h>

// ****************************************************************************
// defines
// ****************************************************************************
  
// External Events for RTOS application   
#define UART_DATA_EVT                     0x0010
#define UART_ERROR_EVT                    0x0020
  
// ****************************************************************************
// typedefs
// ****************************************************************************

//! \brief SDI Command IDs.
//!
typedef enum
{
    SDIMSG_Type_ASYNC       = 0x00,
    SDIMSG_Type_SYNCREQ     = 0x01,
    SDIMSG_Type_SYNCRSP     = 0x02
} SDIMSG_Type;


//! \brief Structure to send an SDI message to the stack thread.
//!
typedef struct _npimsg_t
{
    // Msg Type
    uint_least8_t msgType;

    // message buffer size
    uint8_t pBufSize;

    // message buffer
    uint8_t *pBuf;
} SDIMSG_msg_t;


//*****************************************************************************
// globals
//*****************************************************************************

//*****************************************************************************
// function prototypes
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif /* SDIDATA_H */

