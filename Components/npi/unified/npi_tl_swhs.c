/*
 * npi_tl.c
 *
 * NPI Transport Layer API
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
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

#if defined(NPI_USE_SPI)
#include "inc/npi_tl_spi.h"
#elif defined(NPI_USE_UART)
#include "inc/npi_tl_uart_swhs.h"
#else
#error "Must define an underlying serial bus for NPI"
#endif //NPI_USE_SPI

#ifdef POWER_SAVING
// Indexes for pin configurations in PIN_Config array
#define REM_RDY_PIN_IDX      0
#define LOC_RDY_PIN_IDX      1




#define LocRDY_ENABLE()      PIN_setOutputValue(hNpiHandshakePins, locRdyPIN, 0)
#define LocRDY_DISABLE()     PIN_setOutputValue(hNpiHandshakePins, locRdyPIN, 1)
#else
#define LocRDY_ENABLE()
#define LocRDY_DISABLE()
#endif //POWER_SAVING



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

//! \brief The packet that was being sent when MRDY HWI negedge was received
static volatile uint32_t mrdyPktStamp = 0;

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

//! \brief Size of allocated Tx and Rx buffers
uint16_t npiBufSize = 0;
//! \brief State variable describing soft handshaking state
hsState handshakingState = HS_GPIO_STATE;
//! \brief Store a local copy of the current NPI params
static NPITL_Params npiTLParams;
npiTLCallBacks taskCBs;

#ifdef POWER_SAVING
//! \brief PIN Config for Mrdy and Srdy signals without PIN IDs 
static PIN_Config npiHandshakePinsCfg[] =
{
    Board_UART_RX | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN | PIN_PULLUP,
    PIN_TERMINATE
};
//! \brief PIN State for remRdy and locRdy signals
static PIN_State npiHandshakePins;

//! \brief PIN Handles for remRdy and locRdy signals
static PIN_Handle hNpiHandshakePins;
#ifdef NPI_SW_HANDSHAKING_DEBUG
//! \brief PIN State for debugging signal
static PIN_State npiProfilingDebugPin;
//! \brief PIN Handles for debugging signal
static PIN_Handle hNpiProfilingDebugPin;
#endif //NPI_SW_HANDSHAKING_DEBUG

//! \brief No way to detect whether positive or negative edge with PIN Driver
//!        Use a flag to keep track of state
static uint8_t remRdy_state;
#ifdef NPI_SW_HANDSHAKING_DEBUG
static PIN_Config npiProfilingDebugPinCfg[] =
{
    Board_LED1 | PIN_GPIO_OUTPUT_EN,
    PIN_TERMINATE
};
static uint32_t profilingDebugPin = Board_LED1;
#endif //NPI_SW_HANDSHAKING_DEBUG
#endif //POWER_SAVING

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief Call back function provided to underlying serial interface to be
//         invoked upon the completion of a transmission
static void NPITL_transmissionCallBack(uint16_t Rxlen, uint16_t Txlen);

#ifdef POWER_SAVING
//! \brief HWI interrupt function for remRdy
static void NPITL_remRdyPINHwiFxn(PIN_Handle hPin, PIN_Id pinId);
//! \brief callback for software handshaking
void NPITL_chirpRecievedCB(void);
//! \brief This routine is used to set constraints on power manager
static void NPITL_setPM(void);

//! \brief This routine is used to release constraints on power manager
static void NPITL_relPM(void);
#endif //POWER_SAVING

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

    // This will be updated to be able to select SPI/UART TL at runtime
    // Now only compile time with the NPI_USE_[UART,SPI] flag

#if defined(NPI_USE_UART)
#elif defined(NPI_USE_SPI)
    transportOpen(params->portBoardID, 
                  &params->portParams.spiParams, 
                  NPITL_transmissionCallBack);
#endif //NPI_USE_UART
    
	hNpiHandshakePins = PIN_open(&npiHandshakePins, npiHandshakePinsCfg);
	PIN_registerIntCb(hNpiHandshakePins, NPITL_remRdyPINHwiFxn);
	PIN_setConfig(hNpiHandshakePins,
				  PIN_BM_IRQ,
				  Board_UART_RX | PIN_IRQ_BOTHEDGES);

	// Enable wakeup
	PIN_setConfig(hNpiHandshakePins,
				  PINCC26XX_BM_WAKEUP,
				  Board_UART_RX | PINCC26XX_WAKEUP_NEGEDGE);
#ifdef NPI_SW_HANDSHAKING_DEBUG
	hNpiProfilingDebugPin= PIN_open(&npiProfilingDebugPin, npiProfilingDebugPinCfg);
#endif //NPI_SW_HANDSHAKING_DEBUG

	npiTLParams = *params;	//Keep a copy of TLParams local to the TL so that the UART can be closed/reopened
#ifndef POWER_SAVING
    // This call will start repeated Uart Reads when Power Savings is disabled
    transportRead();
#endif 

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
  
    // Close Transport Layer 
    transportClose();
    
#ifdef POWER_SAVING  
    // Close PIN Handle
    PIN_close(hNpiHandshakePins);
#ifdef NPI_SW_HANDSHAKING_DEBUG
    PIN_close(hNpiProfilingDebugPin);
#endif //NPI_SW_HANDSHAKING
    // Release Power Management
    NPITL_relPM();
#endif //POWER_SAVING

    NPIUtil_ExitCS(key);
}
  
// -----------------------------------------------------------------------------
//! \brief      This routine returns the state of transmission on NPI
//!
//! \return     bool - state of NPI transmission - 1 - active, 0 - not active
// -----------------------------------------------------------------------------
bool NPITL_checkNpiBusy(void)
{
	return (handshakingState & HS_WAITFORCHIRP);
}

#ifdef POWER_SAVING
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
#endif //POWER_SAVING

#ifdef POWER_SAVING
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
#endif //POWER_SAVING

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine is used to handle an MRDY transition from a task
//!             context. Certain operations such as UART_read() cannot be
//!             performed from a HWI context
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITL_handleRemRdyEvent(void)
{
	_npiCSKey_t key;
	key = NPIUtil_EnterCS();
	//If the UART port is closed, then open it
    if(HS_GPIO_STATE & handshakingState)
    {
    	NPITL_setPM();
    	//Close the GPIO, then
    	//Open the UART
    	 PIN_close(hNpiHandshakePins);
    	 transportOpen(npiTLParams.portBoardID,
					  &npiTLParams.portParams.uartParams,
					  NPITL_transmissionCallBack, NPITL_chirpRecievedCB);
    	 handshakingState |=  HS_WAITFORCHIRP|HS_UART_STATE;
    	 //Clear GPIO flag, we are no longer in this state
    	 handshakingState &= ~HS_GPIO_STATE;
    }
    else
    {
    	//Once UART is open
        //Handle the RemRdy Event
        transportRemRdyEvent();
    }
    NPIUtil_ExitCS(key);
}

// -----------------------------------------------------------------------------
//! \brief      This is a HWI function handler for the MRDY pin. Some MRDY
//!             functionality can execute from this HWI context. Others
//!             must be executed from task context hence the taskCBs.remRdyCB()
//!
//! \param[in]  hPin - PIN Handle
//! \param[in]  pinId - ID of pin that triggered HWI
//!
//! \return     void
// -----------------------------------------------------------------------------
static void NPITL_remRdyPINHwiFxn(PIN_Handle hPin, PIN_Id pinId)
{ 
    NPITL_setPM();
    // Signal to registered task that Rem Ready signal has changed state
    if (taskCBs.remRdyCB)
    {
        taskCBs.remRdyCB(remRdy_state);
    }
}
// -----------------------------------------------------------------------------
//! \brief      This function is used to trigger the RemRdy Event in the task
//!             from a chirp callBack
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITL_chirpRecievedCB(void)
{
	//Call the RemRdy call back in the task
    if (taskCBs.remRdyCB)
    {
        taskCBs.remRdyCB(remRdy_state);
    }

}
#endif //POWER_SAVING

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
    
    // If Task is registered, invoke transaction complete callback
    if (taskCBs.transCompleteCB)
    {
        taskCBs.transCompleteCB(Rxlen, Txlen);
    }
#ifdef NPI_SW_HANDSHAKING_DEBUG
    //Set the profiling pin high
    PIN_setOutputValue(hNpiProfilingDebugPin, profilingDebugPin, 1);
#endif //NPI_SW_HANDSHAKING_DEBUG
    // Close the UART
    transportClose();
    // Open the Pins for ISR
    hNpiHandshakePins = PIN_open(&npiHandshakePins, npiHandshakePinsCfg);
    	//replace remRdyPIN with Board_UART_RX
    	PIN_registerIntCb(hNpiHandshakePins, NPITL_remRdyPINHwiFxn);
    	PIN_setConfig(hNpiHandshakePins,
    				  PIN_BM_IRQ,
    				  Board_UART_RX | PIN_IRQ_BOTHEDGES);

    	// Enable wakeup
    	PIN_setConfig(hNpiHandshakePins,
    				  PINCC26XX_BM_WAKEUP,
    				  Board_UART_RX | PINCC26XX_WAKEUP_NEGEDGE);
#ifdef NPI_SW_HANDSHAKING_DEBUG
    	//Indicate that we are now asleep in the GPIO state
    	PIN_setOutputValue(hNpiProfilingDebugPin, profilingDebugPin, 0);
#endif //NPI_SW_HANDSHAKING_DEBUG
    	//It is also valid to clear all flags at this point
    	_npiCSKey_t key;
    	key = NPIUtil_EnterCS();
    	handshakingState = HS_GPIO_STATE;
    	NPIUtil_ExitCS(key);
#ifdef POWER_SAVING
    NPITL_relPM();
#endif //POWER_SAVING
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
#ifdef POWER_SAVING
    _npiCSKey_t key;
    key = NPIUtil_EnterCS();
#endif //POWER_SAVING
    
    // Check to make sure NPI is not currently in a transaction
    if (NPITL_checkNpiBusy())
    {
#ifdef POWER_SAVING
        NPIUtil_ExitCS(key);
#endif // POWER_SAVING

        return NPI_BUSY;
    }
    
    // Check to make sure that write size is not greater than what is 
    // allowed
    if (len > npiBufSize)
    {
#ifdef POWER_SAVING
        NPIUtil_ExitCS(key);
#endif // POWER_SAVING

        return NPI_TX_MSG_OVERSIZE;
    }
    
    // Copy into the second byte of npiTxBuf. This will save Serial Port
    // Specific TL code from having to shift one byte later on for SOF. 
    memcpy(&npiTxBuf[1], buf, len);
    npiTxBufLen = len;
    npiTxActive = TRUE;
    txPktCount++;

    transportWrite(npiTxBufLen);

#ifdef POWER_SAVING
    NPIUtil_ExitCS(key);
#endif //POWER_SAVING

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
