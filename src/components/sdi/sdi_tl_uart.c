/******************************************************************************

 @file  sdi_tl_uart.c

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

// ****************************************************************************
// includes
// ****************************************************************************
#include <string.h>
#include <xdc/std.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "ICall.h"
#include "Board.h"
#include "hal_types.h"
#include "bcomdef.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include "inc/sdi_config.h"
#include "inc/sdi_tl_uart.h"
#include "inc/sdi_data.h"

#include <ti/drivers/uart/UARTCC26XX.h>

// ****************************************************************************
// defines
// ****************************************************************************

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************
//! \brief UART Handle for UART Driver
static UART_Handle uartHandle;

//! \brief UART ISR Rx Buffer
static Char isrRxBuf[UART_ISR_BUF_SIZE];

//! \brief SDI TL call back function for the end of a UART transaction
static sdiCB_t sdiTransmitCB = NULL;

#ifdef POWER_SAVING
//! \brief Flag signalling receive in progress
static uint8 RxActive = FALSE;

//! \brief Flag signalling transmit in progress
static uint8 TxActive = FALSE;

//! \brief Value of MRDY SDI TL pin
static uint8 mrdy_flag = 1;
#endif //POWER_SAVING

//! \brief Pointer to SDI TL TX Buffer
static Char* TransportRxBuf;

//! \brief Length of bytes received
static uint16 TransportRxLen = 0;

//! \brief Pointer to SDI TL RX Buffer
static Char* TransportTxBuf;

//! \brief Length of bytes to send from SDI TL Tx Buffer
static uint16 TransportTxLen = 0;

//! \brief UART Object. Initialized in board specific files
extern UARTCC26XX_Object uartCC26XXObjects[];

UART_Params paramsUART;

//! \brief Pointer to Application RX event callback function for optional
//!        rerouting of messages to application.
//!
static sdiTLIncomingEventCBack_t incomingRXErrorStatusAppCBFunc = NULL;

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief UART ISR function. Invoked upon specific threshold of UART RX FIFO size
static uint16 SDITLUART_readIsrBuf(size_t size);

//! \brief UART Callback invoked after UART write completion
static void SDITLUART_writeCallBack(UART_Handle handle, void *ptr, size_t size);

//! \brief UART Callback invoked after readsize has been read or timeout
static void SDITLUART_readCallBack(UART_Handle handle, void *ptr, size_t size);


void SDITLUART_registerIncomingRXErrorStatusAppCB(sdiTLIncomingEventCBack_t appRxErrStatusCB)
{
  incomingRXErrorStatusAppCBFunc = appRxErrStatusCB;
}


void SDITLUART_closeUART(void)
{
  ICall_CSState key;

  key = ICall_enterCriticalSection();

  // Cancel any pending reads
  UART_readCancel(uartHandle);

  // Close / power off the UART.
  UART_close(uartHandle);

  ICall_leaveCriticalSection(key);
}


uint8 SDITLUART_configureUARTParams(UART_Params *initParams)
{
  uint8 status = SUCCESS;
  ICall_CSState key;

  SDITLUART_closeUART();

  key = ICall_enterCriticalSection();

  // Open / power on the UART.
  uartHandle = UART_open(Board_UART, &paramsUART);
  if(uartHandle != NULL)
  {
    //DEBUG("UART_open successful");
  }else{
    //DEBUG("ERROR in UART_open");
    status = FAILURE;
  }

  //Enable Partial Reads on all subsequent UART_read()
  status = UART_control(uartHandle, UARTCC26XX_RETURN_PARTIAL_ENABLE,  NULL);

  ICall_leaveCriticalSection(key);

  #ifndef POWER_SAVING
    //Initiate first read to start polling UART
    SDITLUART_readTransport();
  #endif //POWER_SAVING

  return status;
}

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
void SDITLUART_initializeTransport(Char *tRxBuf, Char *tTxBuf, sdiCB_t sdiCBack)
{
    // Set UART transport callbacks
    TransportRxBuf = tRxBuf;
    TransportTxBuf = tTxBuf;
    sdiTransmitCB = sdiCBack;

    // Configure UART parameters.
    UART_Params_init(&paramsUART);
    paramsUART.baudRate = SDI_UART_BR;
    paramsUART.readDataMode = UART_DATA_BINARY;
    paramsUART.writeDataMode = UART_DATA_BINARY;
    paramsUART.dataLength = UART_LEN_8;
    paramsUART.stopBits = UART_STOP_ONE;
    paramsUART.readMode = UART_MODE_CALLBACK;
    paramsUART.writeMode = UART_MODE_CALLBACK;
    paramsUART.readEcho = UART_ECHO_OFF;

    paramsUART.readCallback = SDITLUART_readCallBack;
    paramsUART.writeCallback = SDITLUART_writeCallBack;

    //paramsUART.readReturnMode = UART_RETURN_FULL;

    // Open / power on the UART.
    uartHandle = UART_open(Board_UART, &paramsUART);
    if(uartHandle != NULL)
    {
      //DEBUG("ERROR in UART_open");
    }
    //Enable Partial Reads on all subsequent UART_read()
    UART_control(uartHandle, UARTCC26XX_RETURN_PARTIAL_ENABLE,  NULL);

    return;
}

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine stops any pending reads
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITLUART_stopTransfer(void)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();

    mrdy_flag = 1;

    // If we have no bytes in FIFO yet we must assume there was nothing to read
    // or that the FIFO has already been read for this UART_read()
    // In either case UART_readCancel will call the read CB function and it will
    // invoke sdiTransmitCB with the appropriate number of bytes read
    if (!UARTCharsAvail(((UARTCC26XX_HWAttrsV1 const *)(uartHandle->hwAttrs))->baseAddr))
    {
        RxActive = FALSE;
        UART_readCancel(uartHandle);
    }

    ICall_leaveCriticalSection(key);
    return;
}
#endif //POWER_SAVING

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine is called from the application context when MRDY is
//!             de-asserted
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITLUART_handleMrdyEvent(void)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();

    mrdy_flag = 0;

    // If we haven't already begun reading, now is the time before Master
    //    potentially starts to send data
    // The !TxActive condition is because we will call UART_sdiRead() prior to setting
    // TxActive true. There is the possibility that MRDY gets set high which
    // clears RxActive prior to us getting to this event. This will cause us to
    // read twice per transaction which will cause the transaction to never
    // complete
    if ( !RxActive && !TxActive )
    {
        SDITLUART_readTransport();
    }

    // If we have something to write, then the Master has signalled it is ready
    //    to receive. Time to write.
    if ( TxActive )
    {
        // Check to see if transport is successful. If not, reset TxLen to allow
        // another write to be processed
        if ( UART_write(uartHandle, TransportTxBuf, TransportTxLen) == UART_ERROR )
        {
          TxActive = FALSE;
          TransportTxLen = 0;
        }
    }

    ICall_leaveCriticalSection(key);

    return;
}
#endif //POWER_SAVING

// -----------------------------------------------------------------------------
//! \brief      This callback is invoked on Write completion
//!
//! \param[in]  handle - handle to the UART port
//! \param[in]  ptr    - pointer to data to be transmitted
//! \param[in]  size   - size of the data
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITLUART_writeCallBack(UART_Handle handle, void *ptr, size_t size)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();
    uint8 errStatus = 0;

    if (errStatus = ((UARTCC26XX_Handle)handle->object)->status)
    {
      //report UART error status to application
      if(incomingRXErrorStatusAppCBFunc != NULL)
        incomingRXErrorStatusAppCBFunc(UART_ERROR_EVT, &errStatus, sizeof(errStatus));
    }

#ifdef POWER_SAVING
    if ( !RxActive )
    {
        UART_readCancel(uartHandle);
        if ( sdiTransmitCB )
        {
            sdiTransmitCB(TransportRxLen,TransportTxLen);
        }
    }

    TxActive = FALSE;

#else
    if ( sdiTransmitCB )
    {
        sdiTransmitCB(0,TransportTxLen);
    }
#endif //POWER_SAVING

    ICall_leaveCriticalSection(key);
}

// -----------------------------------------------------------------------------
//! \brief      This callback is invoked on Read completion of readSize/receive
//!             timeout
//!
//! \param[in]  handle - handle to the UART port
//! \param[in]  ptr    - pointer to buffer to read data into
//! \param[in]  size   - size of the data
//!
//! \return     void
// -----------------------------------------------------------------------------
static void SDITLUART_readCallBack(UART_Handle handle, void *ptr, size_t size)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();
    uint8 errStatus = 0;

    if (errStatus = ((UARTCC26XX_Handle)handle->object)->status)
    {
      //report UART error status to application
      if(incomingRXErrorStatusAppCBFunc != NULL)
        incomingRXErrorStatusAppCBFunc(UART_ERROR_EVT, &errStatus, sizeof(errStatus));
    }

    if (size)
    {
        if (size != SDITLUART_readIsrBuf(size))
        {
            // Buffer overflow imminent. Cancel read and pass to higher layers
            // for handling
#ifdef POWER_SAVING
            RxActive = FALSE;
#endif //POWER_SAVING
            if ( sdiTransmitCB )
            {
                sdiTransmitCB(SDI_TL_BUF_SIZE,TransportTxLen);
            }
        }
    }

#ifdef POWER_SAVING
    // Read has been cancelled by transport layer, or bus timeout and no bytes in FIFO
    //    - do not invoke another read
    if ( !UARTCharsAvail(((UARTCC26XX_HWAttrsV1 const *)(uartHandle->hwAttrs))->baseAddr) &&
            mrdy_flag )
    {
        RxActive = FALSE;

        // If TX has also completed then we are safe to issue call back
        if ( !TxActive && sdiTransmitCB )
        {
            sdiTransmitCB(TransportRxLen,TransportTxLen);
        }
    }
    else
    {
        UART_read(uartHandle, &isrRxBuf[0], UART_ISR_BUF_SIZE);
    }
#else
    if ( sdiTransmitCB )
    {
        sdiTransmitCB(size,0);
    }
    TransportRxLen = 0;
    UART_read(uartHandle, &isrRxBuf[0], UART_ISR_BUF_SIZE);
#endif //POWER_SAVING

    ICall_leaveCriticalSection(key);
}

// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the transport layer based on len,
//!             and places it into the buffer.
//!
//! \param[in]  size - amount of bytes in UART ISR Rx Buffer
//!
//! \return     uint16 - number of bytes read from transport
// -----------------------------------------------------------------------------
static uint16 SDITLUART_readIsrBuf(size_t size)
{
    uint8_t i = 0;

    // Copy the UART buffer to the application buffer
    // Do not allow overflow of buffer. Instead pass up to SDI module and allow
    // it to handle
    for (; (i < size) && (TransportRxLen < SDI_TL_BUF_SIZE); i++)
    {
        TransportRxBuf[TransportRxLen++] = isrRxBuf[i];
        isrRxBuf[i] = 0;
    }

    return i;
}

// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the UART
//!
//! \return     void
// -----------------------------------------------------------------------------
void SDITLUART_readTransport(void)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();

#ifdef POWER_SAVING
    RxActive = TRUE;
#endif //POWER_SAVING

    TransportRxLen = 0;
    UART_read(uartHandle, &isrRxBuf[0], UART_ISR_BUF_SIZE);

    ICall_leaveCriticalSection(key);
}


// -----------------------------------------------------------------------------
//! \brief      This routine writes copies buffer addr to the transport layer.
//!
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint8 - number of bytes written to transport
// -----------------------------------------------------------------------------
uint16 SDITLUART_writeTransport(uint16 len)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();

    TransportTxLen = len;

#ifdef POWER_SAVING
    TxActive = TRUE;

    // Start reading prior to impending write transaction
    // We can only call UART_write() once MRDY has been signaled from Master
    // device
    SDITLUART_readTransport();
#else
    // Check to see if transport is successful. If not, reset TxLen to allow
    // another write to be processed
    if(UART_write(uartHandle, TransportTxBuf, TransportTxLen) == UART_ERROR )
    {
      TransportTxLen = 0;
    }
#endif //POWER_SAVING
    ICall_leaveCriticalSection(key);

    return TransportTxLen;
}
