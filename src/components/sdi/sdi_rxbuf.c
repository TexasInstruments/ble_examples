/******************************************************************************

 @file  sdi_rxbuf.c

  SDI RX Buffer and utilities

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


// ****************************************************************************
// includes
// ****************************************************************************
#include <string.h>
#include <xdc/std.h>

#include "Board.h"
#include "hal_types.h"
#include "inc/sdi_config.h"
#include "inc/sdi_tl.h"

// ****************************************************************************
// defines
// ****************************************************************************
#define SDIRXBUF_RXHEAD_INC(x)   RxBufHead += x;               \
    RxBufHead %= SDI_TL_BUF_SIZE;

#define SDIRXBUF_RXTAIL_INC(x)   RxBufTail += x;               \
    RxBufTail %= SDI_TL_BUF_SIZE;

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//Recieve Buffer for all SDI messages
static uint8 RxBuf[SDI_TL_BUF_SIZE];
static uint16 RxBufHead = 0;
static uint16 RxBufTail = 0;

//*****************************************************************************
// function prototypes
//*****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      SDIRxBuf_Read
//!
//! \param[in]  len -
//!
//! \return     uint16 -
// -----------------------------------------------------------------------------
uint16 SDIRxBuf_Read(uint16 len)
{
    uint16 partialLen = 0;

    // Need to make two reads due to wrap around of circular buffer
    if ((len + RxBufTail) > SDI_TL_BUF_SIZE)
    {
        partialLen = SDI_TL_BUF_SIZE - RxBufTail;
        SDITL_readTL(&RxBuf[RxBufTail],partialLen);
        len -= partialLen;
        RxBufTail = 0;
    }

    // Read remainder of data from Transport Layer
    SDITL_readTL(&RxBuf[RxBufTail],len);
    SDIRXBUF_RXTAIL_INC(len);

    // Return len to original size
    len += partialLen;

    return len;
}

// -----------------------------------------------------------------------------
//! \brief      Returns number of bytes that are unparsed in RxBuf
//!
//! \return     uint16 -
// -----------------------------------------------------------------------------
uint16 SDIRxBuf_GetRxBufCount(void)
{
    return ((RxBufTail - RxBufHead) + SDI_TL_BUF_SIZE) % SDI_TL_BUF_SIZE;
}

// -----------------------------------------------------------------------------
//! \brief      Returns number of bytes that are available in RxBuf
//!
//! \return     uint16 -
// -----------------------------------------------------------------------------
uint16 SDIRxBuf_GetRxBufAvail(void)
{
    return (SDI_TL_BUF_SIZE - SDIRxBuf_GetRxBufCount());
}

// -----------------------------------------------------------------------------
//! \brief      SDIRxBuf_ReadFromRxBuf
//!
//! \return     uint16 -
// -----------------------------------------------------------------------------
uint16 SDIRxBuf_ReadFromRxBuf(uint8_t *buf, uint16 len)
{
    uint16_t idx;
    for (idx = 0; idx < len; idx++)
    {
        *buf++ = RxBuf[RxBufHead];
        SDIRXBUF_RXHEAD_INC(1)
    }

    return len;
}
