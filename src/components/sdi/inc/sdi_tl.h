//******************************************************************************
//! \file           sdi_tl.h
//! \brief          SDI Transport Layer API
//
//   Revised        $Date: 2015-01-29 11:51:00 -0800 (Thu, 29 Jan 2015) $
//   Revision:      $Revision: 42121 $
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
