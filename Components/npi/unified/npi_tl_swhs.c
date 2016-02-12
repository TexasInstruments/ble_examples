/*
 * npi_tl_swhs.c
 *
 * NPI Transport Layer API
 *
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
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "Board.h"
#include "hal_types.h"

#include "inc/npi_tl_swhs.h"
#include "inc/npi_data_swhs.h"
#include "inc/npi_util.h"

// ****************************************************************************
// defines
// ****************************************************************************

#include "inc/npi_tl_uart_swhs.h"

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//! \brief Flag for low power mode
static volatile bool npiPMSetConstraint = FALSE;

//! \brief Flag for ongoing NPI TX
static volatile bool npiTxActive = FALSE;

//! \brief Packets transmitted counter
static uint32_t txPktCount = 0;

//! \brief NPI Transport Layer receive buffer
uint8_t *npiRxBuf;

//! \brief Index to last byte written into NPI Transport Layer receive buffer
static uint16_t npiRxBufTail = 0;

//! \brief Index to first byte to read from NPI Transport Layer receive buffer
static uint16_t npiRxBufHead = 0;

//! \brief NPI Transport Layer transmit buffer
uint8_t *npiTxBuf;

//! \brief Number of bytes in NPI Transport Layer transmit buffer
static uint16_t npiTxBufLen = 0;
//! \brief Store a local copy of the current NPI params
static NPITL_Params npiTLParams;
//! \brief UART RX pin configuration
static PIN_Config npiUartRxPinCfg[] =
{
    Board_UART_RX | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    PIN_TERMINATE
};
//! \brief PIN State for UART Rx pin, used when in HS_GPIO_MODE
static PIN_State npiUartRxPin;

//! \brief PIN Handles for uartRxPin that is used as the SWHS pin
static PIN_Handle hNpiUartRxPin;

#ifdef SWHS_DEBUG
static PIN_Config npiProfilingPinCfg[] =
{
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_INPUT_DIS | PIN_PULLUP,
    PIN_TERMINATE
};
//! \brief PIN State for profiling pin that toggles during critical sections
static PIN_State npiProfilingPin;

//! \brief PIN Handles for profiling pin that toggles during critical sections
static PIN_Handle hNpiProfilingPin;
#endif //SWHS_DEBUG

//! \brief State variable that tracks TL status for npi_task
static tlState trasnportLayerState = TL_closed;

//! \brief Size of allocated Tx and Rx buffers
uint16_t npiBufSize = 0;
//! \brief Store a local copy of the current NPI params
static NPITL_Params npiTLParams;

npiTLCallBacks taskCBs;


//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief Call back function provided to underlying serial interface to be
//         invoked upon the completion of a transmission
static void NPITL_transmissionCallBack(uint16_t Rxlen, uint16_t Txlen);
static void NPITL_handshakeCompleteCallBack(hsTransactionRole role);

//! \brief HWI interrupt function for remRdy
static void NPITL_rxPinHwiFxn(PIN_Handle hPin, PIN_Id pinId);

//! \brief This routine is used to set constraints on power manager
static void NPITL_setPM(void);

//! \brief This routine is used to release constraints on power manager
static void NPITL_relPM(void);


// -----------------------------------------------------------------------------
//! \brief      This routine initializes the transport layer and opens the port
//!             of the device. Note that based on project defines, either the
//!             UART, or SPI driver can be used.
//!
//! \param[in]  params - Transport Layer parameters
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITL_openTL(NPITL_Params *params)
{
    _npiCSKey_t key;
    key = NPIUtil_EnterCS();
    
    // Set NPI Task Call backs
    memcpy(&taskCBs, &params->npiCallBacks, sizeof(params->npiCallBacks));
    
    // Allocate memory for Transport Layer Tx/Rx buffers
    npiBufSize = params->npiTLBufSize;
    npiRxBuf = NPIUTIL_MALLOC(params->npiTLBufSize);
    memset(npiRxBuf, 0, npiBufSize);
    npiTxBuf = NPIUTIL_MALLOC(params->npiTLBufSize);
    memset(npiTxBuf, 0, npiBufSize);
    
    
    hNpiUartRxPin = PIN_open(&npiUartRxPin, npiUartRxPinCfg);
    PIN_registerIntCb(hNpiUartRxPin, NPITL_rxPinHwiFxn);
    PIN_setConfig(hNpiUartRxPin, 
                  PIN_BM_IRQ, 
                  Board_UART_RX | PIN_IRQ_BOTHEDGES);

    // Enable wakeup
    PIN_setConfig(hNpiUartRxPin, 
                  PINCC26XX_BM_WAKEUP, 
                  Board_UART_RX | PINCC26XX_WAKEUP_NEGEDGE);
    //Note that open TL is only called when NPI task is being initialized
    //transportLayerState variable defaults to closed.
#ifdef SWHS_DEBUG  
    //Open Profiling Pin if in debug mode 
    hNpiProfilingPin = PIN_open(&npiProfilingPin, npiProfilingPinCfg);
#endif //SWHS_DEBUG
    //Keep a copy of TLParams local to the TL so that the UART can be closed/reopened
    npiTLParams = *params;
    //Here we will initialize the transport which will setup the callbacks
    //This call does not open the UART
    transportInit( &npiTLParams.portParams.uartParams, 
                   NPITL_transmissionCallBack,
                   NPITL_handshakeCompleteCallBack);
    
    NPIUtil_ExitCS(key);
}

// -----------------------------------------------------------------------------
//! \brief      This routine closes the transport layer
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITL_closeTL(void)
{
    _npiCSKey_t key;
    key = NPIUtil_EnterCS();
    
    // Clear NPI Task Call backs
    memset(&taskCBs, 0, sizeof(taskCBs));
  
    // Free Transport Layer RX/TX buffers
    npiBufSize = 0;
    NPIUTIL_FREE(npiRxBuf);
    NPIUTIL_FREE(npiTxBuf);
  
    // Close the RxPin
    PIN_close(hNpiUartRxPin);
    // Close UART transport Layer 
    transportClose();
    trasnportLayerState = TL_closed;
    
    NPITL_relPM();

    NPIUtil_ExitCS(key);
}
// -----------------------------------------------------------------------------
//! \brief      This routine calls the UART transport open
//!
//! \param[in]  initiatorState - Whether the UART is being opened by initiator
//!             or responder. 1 - initiator 0 - responsder
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITL_openTransportPort(hsTransactionRole role)
{
  NPITL_setPM();
  // Close the RxPin
  PIN_close(hNpiUartRxPin);
  transportOpen(npiTLParams.portBoardID,
                &npiTLParams.portParams.uartParams, role);
  trasnportLayerState = TL_busy;
}
// -----------------------------------------------------------------------------
//! \brief      This routine returns the state of transmission on NPI
//!
//! \return     bool - state of NPI transmission - 1 - active, 0 - not active
// -----------------------------------------------------------------------------
tlState NPITL_getTlStatus(void)
{
  return trasnportLayerState;
}
// -----------------------------------------------------------------------------
//! \brief      This routine is used to set constraints on power manager
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITL_setPM(void)
{
    if (npiPMSetConstraint)
    {
        return;
    }

    // set constraints for Standby and idle mode
    Power_setConstraint(Power_SB_DISALLOW);
    Power_setConstraint(Power_IDLE_PD_DISALLOW);
    npiPMSetConstraint = TRUE;
}
// -----------------------------------------------------------------------------
//! \brief      This routine is used to release constraints on power manager
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITL_relPM(void)
{
    if (!npiPMSetConstraint)
    {
        return;
    }

    // release constraints for Standby and idle mode
    Power_releaseConstraint(Power_SB_DISALLOW);
    Power_releaseConstraint(Power_IDLE_PD_DISALLOW);
    npiPMSetConstraint = FALSE;
}
// -----------------------------------------------------------------------------
//! \brief      This is a HWI function handler for the UART Rx pin. It will wake
//!             the TL and signal the app to open the TL by closing UART and 
//!             opening the GPIO
//!
//! \param[in]  hPin - PIN Handle
//! \param[in]  pinId - ID of pin that triggered HWI
//!
//! \return     void
// -----------------------------------------------------------------------------
static void NPITL_rxPinHwiFxn(PIN_Handle hPin, PIN_Id pinId)
{
    NPITL_setPM();
    // Signal to registered task that TL is awake, needs to open
    if (taskCBs.tlOpenCB)
    {
        taskCBs.tlOpenCB();
    }
}

// -----------------------------------------------------------------------------
//! \brief      This callback is invoked on the completion of one transmission
//!             to/from the host MCU. Any bytes receives will be [0,Rxlen) in
//!             npiRxBuf.
//!             If bytes were receives or transmitted, this function notifies
//!             the NPI task via registered call backs
//!
//! \param[in]  Rxlen   - length of the data received
//! \param[in]  Txlen   - length of the data transferred
//!
//! \return     void
// -----------------------------------------------------------------------------
static void NPITL_transmissionCallBack(uint16_t Rxlen, uint16_t Txlen)
{
    npiRxBufHead = 0;
    npiRxBufTail = Rxlen;
    npiTxActive = FALSE;
    //Since the largest valid NPI packet size is 4096
    //valid RxLen fields should only be up to 0x0FFF
    //a larger RxLen value tells the TL that a Rx is in progress
    //and the UART cannot be closed yet
    //in the above case, a CB will be triggered for the Tx, but Rx will wait
    //until ReadCB completes at NPITLUART layer
    if(!(Rxlen & 0x1000))
    {
      //Since we have rx/tx'd a complete packet, it is time to close out the TL
      //and ready the processor for sleep
#ifdef SWHS_DEBUG  
      //Set Pin if in debug mode 
      PIN_setOutputValue(hNpiProfilingPin, Board_LED2, 1);
#endif //SWHS_DEBUG
      transportClose();
      // Open the Pins for ISR
      hNpiUartRxPin = PIN_open(&npiUartRxPin, npiUartRxPinCfg);
      PIN_registerIntCb(hNpiUartRxPin, NPITL_rxPinHwiFxn);
      PIN_setConfig(hNpiUartRxPin, 
                    PIN_BM_IRQ, 
                    Board_UART_RX | PIN_IRQ_BOTHEDGES);

      // Enable wakeup
      PIN_setConfig(hNpiUartRxPin, 
                    PINCC26XX_BM_WAKEUP, 
                    Board_UART_RX | PINCC26XX_WAKEUP_NEGEDGE);
  #ifdef SWHS_DEBUG  
      //Set Pin if in debug mode 
      PIN_setOutputValue(hNpiProfilingPin, Board_LED2, 0);
  #endif //SWHS_DEBUG
      //It is also valid to clear all flags at this point
      trasnportLayerState = TL_closed;
      
        // If Task is registered, invoke transaction complete callback
      if (taskCBs.transCompleteCB)
      {
          taskCBs.transCompleteCB(Rxlen, Txlen);
      }
      NPITL_relPM();
    }
    else
    {
      //be sure to indicate TL is still busy
      trasnportLayerState = TL_busy;
      // If Task is registered, invoke transaction complete callback
      //note that RxLen is zero because the read is incomplete
      if (taskCBs.transCompleteCB)
      {
          taskCBs.transCompleteCB(0, Txlen);
      }
      
    }
}
// -----------------------------------------------------------------------------
//! \brief      This routine notifies the app layer that the HS is complete
//!
//! \return     void
// -----------------------------------------------------------------------------
static void NPITL_handshakeCompleteCallBack(hsTransactionRole role)
{
    //If we are the initiator, give the app the signal to send the message
    //else if not the initiator, our TL is busy because we are listening(reading) 
    //whatever the initiator wanted to tell us
    if(HS_INITIATOR == role)
      trasnportLayerState = TL_ready;
    else 
      trasnportLayerState = TL_busy;
    
    //Trigger the HS complete callback in the task
    if (taskCBs.handshakeCompleteCB)
    {
        taskCBs.handshakeCompleteCB(role);
    }
}
// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the transport layer based on len,
//!             and places it into the buffer.
//!
//! \param[in]  buf - Pointer to buffer to place read data.
//! \param[in]  len - Number of bytes to read.
//!
//! \return     uint16_t - the number of bytes read from transport
// -----------------------------------------------------------------------------
uint16_t NPITL_readTL(uint8_t *buf, uint16_t len)
{
    // Only copy the lowest number between len and bytes remaining in buffer
    len = (len > NPITL_getRxBufLen()) ? NPITL_getRxBufLen() : len;

    memcpy(buf, &npiRxBuf[npiRxBufHead], len);
    npiRxBufHead += len;
    return len;
}

// -----------------------------------------------------------------------------
//! \brief      This routine writes data from the buffer to the transport layer.
//!
//! \param[in]  buf - Pointer to buffer to write data from.
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint16_t - NPI error code value
// -----------------------------------------------------------------------------
uint8_t NPITL_writeTL(uint8_t *buf, uint16_t len)
{
    _npiCSKey_t key;
    key = NPIUtil_EnterCS(); 
    // Check to make sure NPI is not currently in a transaction
    if (TL_ready != NPITL_getTlStatus())
    {
        NPIUtil_ExitCS(key);
        return NPI_BUSY;
    }
    
    // Check to make sure that write size is not greater than what is 
    // allowed
    if (len > npiBufSize)
    {
        NPIUtil_ExitCS(key);
        return NPI_TX_MSG_OVERSIZE;
    }
    // Copy into the second byte of npiTxBuf. This will save Serial Port
    // Specific TL code from having to shift one byte later on for SOF.
    //memset(npiTxBuf, 0, npiTLParams.npiTLBufSize);
    memcpy(&npiTxBuf[1], buf, len);
    npiTxBufLen = len;
    npiTxActive = TRUE;
    txPktCount++;
    trasnportLayerState = TL_busy;
    transportWrite(npiTxBufLen);
    NPIUtil_ExitCS(key);
    return NPI_SUCCESS;
}

// -----------------------------------------------------------------------------
//! \brief      This routine returns the max size receive buffer.
//!
//! \return     uint16_t - max size of the receive buffer
// -----------------------------------------------------------------------------
uint16_t NPITL_getMaxRxBufSize(void)
{
    return(npiBufSize);
}

// -----------------------------------------------------------------------------
//! \brief      This routine returns the max size transmit buffer.
//!
//! \return     uint16_t - max size of the transmit buffer
// -----------------------------------------------------------------------------
uint16_t NPITL_getMaxTxBufSize(void)
{
    return(npiBufSize);
}

// -----------------------------------------------------------------------------
//! \brief      Returns number of bytes that are unread in RxBuf
//!
//! \return     uint16_t - number of unread bytes
// -----------------------------------------------------------------------------
uint16_t NPITL_getRxBufLen(void)
{
    return ((npiRxBufTail - npiRxBufHead) + npiBufSize) % npiBufSize;
}
