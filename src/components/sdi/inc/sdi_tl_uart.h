/******************************************************************************

 @file  sdi_tl_uart.h

 SDI Transport Layer Module for UART

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
#ifndef SDI_TL_UART_H
#define SDI_TL_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************
#include <ti/drivers/UART.h>
// ****************************************************************************
// defines
// ****************************************************************************

#define SDI_UART_FC_THRESHOLD 48
#define SDI_UART_IDLE_TIMEOUT 6
#define SDI_UART_INT_ENABLE TRUE

#if !defined(SDI_UART_BR)
#define SDI_UART_BR 115200
#endif // !SDI_UART_BR

// UART ISR Buffer define
#define UART_ISR_BUF_SIZE 128
#define UART_ISR_BUF_CNT 2
  
// ****************************************************************************
// typedefs
// ****************************************************************************
  // -----------------------------------------------------------------------------
//! \brief      Typedef for call back function mechanism to notify SDI TL that
//!             an SDI transaction has occured
//! \param[in]  uint16     number of bytes received
//! \param[in]  uint16     number of bytes transmitted      
//!
//! \return     void
// ----------------------------------------------------------------------------- 
typedef void (*sdiCB_t)(uint16 Rxlen, uint16 Txlen);

// -----------------------------------------------------------------------------
//! \brief      Typedef for call back function mechanism to reroute incoming SDI
//!             messages.
//!             NOTE: Definer MUST copy contents to local buffer.  SDI task will
//!             free this memory.
//!             NOTE: The contained message buffer does NOT include any "framing"
//!             bytes, ie. SOF, FCS etc.
//! \param[in]  pMsg   Pointer to "unframed" message buffer.
//!
//! \return     void
// -----------------------------------------------------------------------------
typedef void (*sdiTLIncomingEventCBack_t)(uint8_t event, uint8_t *pMsg, uint8_t len);

//*****************************************************************************
// globals
//*****************************************************************************
extern UART_Params paramsUART;
//*****************************************************************************
// function prototypes
//*****************************************************************************

uint8 SDITLUART_configureUARTParams(UART_Params *initParams);

// -----------------------------------------------------------------------------
//! \brief      This routine initializes the transport layer and opens the port
//!             of the device.
//!
//! \param[in]  tRxBuf - pointer to SDI TL Tx Buffer
//! \param[in]  tTxBuf - pointer to SDI TL Rx Buffer
//! \param[in]  sdiCBack - SDI TL call back function to be invoked at the end of 
//!             a UART transaction                     
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITLUART_initializeTransport(Char *tRxBuf, Char *tTxBuf, sdiCB_t sdiCBack);

void SDITLUART_closeUART(void);

// -----------------------------------------------------------------------------
//! \brief      Register callback function to reroute incoming (from UART)
//!             SDI messages.
//!
//! \param[in]  appRxCB   Callback fucntion.
//!
//! \return     void
// -----------------------------------------------------------------------------
extern void SDITLUART_registerIncomingRXErrorStatusAppCB(sdiTLIncomingEventCBack_t appRxErrStatusCB);


// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the UART
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITLUART_readTransport(void);

// -----------------------------------------------------------------------------
//! \brief      This routine writes copies buffer addr to the transport layer.
//!
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint16 - number of bytes written to transport
// -----------------------------------------------------------------------------
uint16 SDITLUART_writeTransport(uint16);

// -----------------------------------------------------------------------------
//! \brief      This routine stops any pending reads
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITLUART_stopTransfer(void);

// -----------------------------------------------------------------------------
//! \brief      This routine is called from the application context when MRDY is
//!             de-asserted
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITLUART_handleMrdyEvent(void);



#ifdef __cplusplus
}
#endif

#endif /* SDI_TL_UART_H */
