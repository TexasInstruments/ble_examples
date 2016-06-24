/******************************************************************************

 @file  sdi_tl.h

  SDI Transport Layer API

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

#ifndef SDI_TL_H
#define SDI_TL_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************
#include "hal_types.h"
#include "inc/sdi_config.h"

// ****************************************************************************
// defines
// ****************************************************************************


#if defined(SDI_USE_UART)
#define transportInit SDITLUART_initializeTransport
#define transportRead SDITLUART_readTransport
#define transportWrite SDITLUART_writeTransport
#define transportStopTransfer SDITLUART_stopTransfer
#define transportMrdyEvent SDITLUART_handleMrdyEvent
#elif defined(NPI_USE_SPI)
#define transportInit NPITLSPI_initializeTransport
#define transportRead NPITLSPI_readTransport
#define transportWrite NPITLSPI_writeTransport
#define transportStopTransfer NPITLSPI_stopTransfer
#define transportMrdyEvent NPITLSPI_handleMrdyEvent
#endif

// ****************************************************************************
// typedefs
// ****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      Typedef for call back function mechanism to notify SDI Task that
//!             an SDI transaction has occured
//! \param[in]  int     number of bytes either received or transmitted
//!
//! \return     void
// -----------------------------------------------------------------------------  
typedef void (*sdiRtosCB_t)(int size);

// -----------------------------------------------------------------------------
//! \brief      Typedef for call back function mechanism to notify SDI Task that
//!             an MRDY edge has occured
//! \param[in]  void
//! \return     void
// -----------------------------------------------------------------------------
typedef void (*sdiMrdyRtosCB_t)();

//*****************************************************************************
// globals
//*****************************************************************************

//*****************************************************************************
// function prototypes
//*****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      This routine initializes the transport layer and opens the port
//!             of the device. Note that based on project defines, either the
//!             UART, USB (CDC), or SPI driver can be used.
//!
//! \param[in]  sdiCBTx - Call back function for TX complete event
//! \param[in]  sdiCBRx - Call back function for RX event
//! \param[in]  sdiCBMrdy - Call back function for MRDY event
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITL_initTL(sdiRtosCB_t sdiCBTx, sdiRtosCB_t sdiCBRx, sdiRtosCB_t sdiCBMrdy);

// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the transport layer based on len,
//!             and places it into the buffer.
//!
//! \param[in]  buf - Pointer to buffer to place read data.
//! \param[in]  len - Number of bytes to read.
//!
//! \return     uint16 - the number of bytes read from transport
// -----------------------------------------------------------------------------
uint16 SDITL_readTL(uint8 *buf, uint16 len);

// -----------------------------------------------------------------------------
//! \brief      This routine writes data from the buffer to the transport layer.
//!
//! \param[in]  buf - Pointer to buffer to write data from.
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint16 - the number of bytes written to transport
// -----------------------------------------------------------------------------
uint16 SDITL_writeTL(uint8 *buf, uint16 len);

// -----------------------------------------------------------------------------
//! \brief      This routine is used to handle an MRDY edge from the application
//!             context. Certain operations such as UART_read() cannot be
//!             performed from the actual MRDY hwi handler
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITL_handleMrdyEvent(void);

// -----------------------------------------------------------------------------
//! \brief      This routine returns the max size receive buffer.
//!
//! \return     uint16 - max size of the receive buffer
// -----------------------------------------------------------------------------
uint16 SDITL_getMaxRxBufSize(void);

// -----------------------------------------------------------------------------
//! \brief      This routine returns the max size transmit buffer.
//!
//! \return     uint16 - max size of the transmit buffer
// -----------------------------------------------------------------------------
uint16 SDITL_getMaxTxBufSize(void);

// -----------------------------------------------------------------------------
//! \brief      Returns number of bytes that are unread in RxBuf
//!
//! \return     uint16 - number of unread bytes
// -----------------------------------------------------------------------------
uint16 SDITL_getRxBufLen(void);

// -----------------------------------------------------------------------------
//! \brief      This routine returns the state of transmission on SDI
//!
//! \return     bool - state of SDI transmission - 1 - active, 0 - not active
// -----------------------------------------------------------------------------
bool SDITL_checkSdiBusy(void);

/*******************************************************************************
 */

#ifdef __cplusplus
}
#endif

#endif /* SDI_TL_H */
