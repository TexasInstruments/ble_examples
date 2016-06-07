//******************************************************************************
//! \file           sdi_tl_uart.h
//! \brief          SDI Transport Layer Module for UART
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
#define UART_ISR_BUF_SIZE 16
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
