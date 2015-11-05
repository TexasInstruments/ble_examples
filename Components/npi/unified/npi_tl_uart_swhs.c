/*
 * npi_tl_uart_swhs.c
 *
 * NPI Transport Layer Module for UART with Software Handshake
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
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "Board.h"
#include "hal_types.h"
#include "hal_defs.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include "inc/npi_util.h"
#include "inc/npi_data_swhs.h"
#include "inc/npi_tl_uart_swhs.h"
#include <ti/drivers/UART.h>

// ****************************************************************************
// defines
// ****************************************************************************

//! \brief NPI UART Message Indexes and Constants
//
#define NPI_UART_MSG_NON_PAYLOAD_LEN                 0x05
#define NPI_UART_MSG_SOF_LEN                         0x01
#define NPI_UART_MSG_HDR_LEN                         0x04
#define NPI_UART_MSG_SOF                             0xFE
#define NPI_UART_MSG_SOF_IDX                         0x00




//! \brief NPI UART Read States
typedef enum
{
  NPITLUART_READ_SOF = 0x00,
  NPITLUART_READ_CHIRP,
  NPITLUART_READ_HDR,
  NPITLUART_READ_PLD,
  NPITLUART_IGNORE
} npiTLUart_readState;

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************
//! \brief UART Handle for UART Driver
static UART_Handle uartHandle;

//! \brief NPI TL call back function for the end of a UART transaction
static npiCB_t npiTransmitCB = NULL;
static npiChirpCB_t npiChirpCB = NULL;

#ifdef POWER_SAVING
//! \brief Flag signalling receive in progress
static uint8_t RxActive = FALSE;

//! \brief Flag signalling transmit in progress
static uint8_t TxActive = FALSE;
#endif //POWER_SAVING

//! \brief Length of bytes received
static uint16_t TransportRxLen = 0;
//! \brief chirp byte
static uint8_t chirp = CHIRP_BYTE;
//! \brief Length of bytes to send from NPI TL Tx Buffer
static uint16_t TransportTxLen = 0;

//! \brief NPI Transport Layer Buffer variables defined in npi_tl.c
extern uint8_t *npiRxBuf;
extern uint8_t *npiTxBuf;
extern uint16_t npiBufSize;
//! \brief NPI Transport Layer State variable defined in npi_tl.c
extern hsState handshakingState;

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief UART Callback invoked after UART write completion
static void NPITLUART_writeCallBack(UART_Handle handle, void *ptr, size_t size);

//! \brief UART Callback invoked after readsize has been read or timeout
static void NPITLUART_readCallBack(UART_Handle handle, void *ptr, size_t size);

//! \brief Check for whether a complete and valid packet has been received
static uint8_t NPITLUART_validPacketFound();

//! \brief Calculate FCS over the given length of buf
static uint8_t NPITLUART_calcFCS(uint8_t *buf, uint16_t len);

// -----------------------------------------------------------------------------
//! \brief      This routine initializes the transport layer and opens the port
//!             of the device.
//!
//! \param[in]  portID     		ID value for board specific UART port
//! \param[in]  portParams  	Parameters used to initialize UART port
//! \param[in]  npiCBack    	Trasnport Layer call back function for packet complete
//! \param[in]  npiChirpCBack   Trasnport Layer call back function  for chirp
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_openTransport(uint8_t portID, UART_Params *portParams,
                             npiCB_t npiCBack, npiChirpCB_t npiChirpCBack)
{
  npiTransmitCB = npiCBack;
  npiChirpCB = npiChirpCBack;

  // Add call backs UART parameters.
  portParams->readCallback = NPITLUART_readCallBack;
  portParams->writeCallback = NPITLUART_writeCallBack;

  // Open / power on the UART.
  uartHandle = UART_open(portID, portParams);
  //Always read after opening
  if(NULL != uartHandle)
  {
	  //Read to be sure we don't miss a chirp or SOF
	  UART_read(uartHandle, npiRxBuf, CHIRP_SIZE);
	  //Write to chirp back to the Remote proc to indicate we're awake
	  handshakingState |= HS_CHIRP_SENT;
	  UART_write(uartHandle, &chirp, CHIRP_SIZE);
  }
}

// -----------------------------------------------------------------------------
//! \brief      This routine closes Transport Layer port           
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_closeTransport(void)
{
  UART_readCancel(uartHandle);
  UART_close(uartHandle);
}

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine stops any pending reads
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_stopTransfer(void)
{
  // This is a dummy function for the UART Master implementation
  // The transfer will end once the master has finished sending and has 
  // received a valid packet from Slave or no data at all
}
#endif //POWER_SAVING

#ifdef POWER_SAVING
// -----------------------------------------------------------------------------
//! \brief      This routine is called from the application context when REM RDY
//!             is de-asserted
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_handleRemRdyEvent(void)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();

  // If read has not yet been started, now is the time before Master
  // potentially starts to send data
  // There is the possibility that MRDY gets set high which
  // clears RxActive prior to us getting to this event. This will cause us to
  // read twice per transaction which will cause the transaction to never
  // complete
  if (!RxActive && !TxActive)
  {
    NPITLUART_readTransport();
  }

  // If write has already been initialized then kick off the driver write
  // now that Master has signalled it is ready
  if (TxActive)
  {
    // Check to see if transport is successful. If not, reset TxLen to allow
    // another write to be processed
    if (UART_write(uartHandle, npiTxBuf, TransportTxLen) == UART_ERROR)
    {
      TxActive = FALSE;
      TransportTxLen = 0;
    }
  }

  NPIUtil_ExitCS(key);
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
void NPITLUART_writeCallBack(UART_Handle handle, void *ptr, size_t size)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
#ifdef POWER_SAVING
  // If we have received a valid packet or have not received any data yet
  // we can end this transaction
  //Note that we want to ignore the writeCallback for Chirps,
  //so we check this flag and clear it and the end of this callback
  if(!(handshakingState & HS_CHIRP_SENT))
  {
	  if (!RxActive || TransportRxLen == 0)
	  {
		UART_readCancel(uartHandle);
		RxActive = FALSE;

		if (npiTransmitCB)
		{
		  if (NPITLUART_validPacketFound() == NPI_SUCCESS)
		  {
			// Decrement as to not include trailing FCS byte
			TransportRxLen--;
		  }
		  else
		  {
			// Did not receive valid packet so denote RX length as zero in CB
			TransportRxLen = 0;
		  }

			  npiTransmitCB(TransportRxLen,TransportTxLen);
		}
	  }
  }


  TxActive = FALSE;
#else
  if (npiTransmitCB)
  {
    npiTransmitCB(0,TransportTxLen);
  }
#endif //POWER_SAVING
  //If we just got here by sending a chirp, clear the chirp flag
  if(handshakingState & HS_CHIRP_SENT)
	  handshakingState &= ~HS_CHIRP_SENT;
  NPIUtil_ExitCS(key);
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
void NPITLUART_readCallBack(UART_Handle handle, void *ptr, size_t size)
{
  static npiTLUart_readState readState = NPITLUART_READ_SOF;
  static uint16_t payloadLen = 0;
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  //If we just recieved a chirp, go into chirp state
  if(size == CHIRP_SIZE && npiRxBuf[0] == CHIRP_BYTE)
	  readState = NPITLUART_READ_CHIRP;

  switch (readState)
  {
  	case NPITLUART_READ_CHIRP:

		if(npiChirpCB)
		{
			readState = NPITLUART_READ_SOF;
			npiChirpCB();
		}
		break;
    case NPITLUART_READ_SOF:
      // Should only have read one byte in this state. 
      if (size == NPI_UART_MSG_SOF_LEN && npiRxBuf[0] == NPI_UART_MSG_SOF)
      {
        // Recevied SOF, Read HDR next. Do not save SOF byte
        UART_read(uartHandle, npiRxBuf, NPI_UART_MSG_HDR_LEN);
        readState = NPITLUART_READ_HDR;
        
      }
      break;
    
    case NPITLUART_READ_HDR:
      if (size == NPI_UART_MSG_HDR_LEN)
      {
        // Header has been read. Increment RxLen
        TransportRxLen += size;
        
        // Determine length of remainder of packet, add an extra byte for FCS
        payloadLen = BUILD_UINT16(npiRxBuf[0],npiRxBuf[1]) + 1;
        
        // Check to see if payload can fit in the remainder of the RxBuf
        if (payloadLen <= npiBufSize - TransportRxLen)
        {
          // Read remainder of packet
          UART_read(uartHandle, &npiRxBuf[TransportRxLen], payloadLen);
          readState = NPITLUART_READ_PLD;
        }
        else
        {
          // Read remainder of packet bytes but ignore them
          UART_read(uartHandle, npiRxBuf, npiBufSize);
          readState = NPITLUART_IGNORE;
        }
      }
      else
      {
        // Error has occured. Reset read state
        readState = NPITLUART_READ_SOF;
      }
    break;
    
    case NPITLUART_READ_PLD:
      if (payloadLen == size)
      {
        // All bytes are read
        TransportRxLen += size;
        
        // Check if FCS is valid
        if (NPITLUART_validPacketFound() == NPI_SUCCESS)
        {
          // Valid Packet. Decrement RxLen to not include FCS since it is
          // checked and valid
          TransportRxLen--;
          
#ifdef POWER_SAVING
          RxActive = FALSE;
          
          // If TX has also completed then we are safe to issue call back
          if (!TxActive && npiTransmitCB)
          {
            npiTransmitCB(TransportRxLen,TransportTxLen);
          }
#else
          if (npiTransmitCB) 
          {
            npiTransmitCB(TransportRxLen,0);
          }
#endif //POWER_SAVING
        }
      }

      // Reset State. Full packet has been read or error has occurred
      readState = NPITLUART_READ_SOF;
      break;
    
    case NPITLUART_IGNORE:
      if (payloadLen == size)
      {
        // All bytes of oversized payload have been read. Reset state
        readState = NPITLUART_READ_SOF;
      }
      else
      {
        // Bytes remaining to be read
        payloadLen -= size;
        
        if (payloadLen > npiBufSize)
        {
          UART_read(uartHandle, npiRxBuf, npiBufSize);
        }
        else
        {
          UART_read(uartHandle, npiRxBuf, payloadLen);
        }
      }
      break;
    
    default:
      // Should not get here. If so reset read state
      readState = NPITLUART_READ_SOF;
      break;
  }

#ifndef POWER_SAVING
  // Initiate next read of SOF byte
  if (readState == NPITLUART_READ_SOF)
  {
    TransportRxLen = 0;
    UART_read(uartHandle, npiRxBuf, NPI_UART_MSG_SOF_LEN);
  }
#endif //!POWER_SAVING
  
  NPIUtil_ExitCS(key);
}

// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the UART
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLUART_readTransport(void)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  
#ifdef POWER_SAVING
  RxActive = TRUE;
#endif //POWER_SAVING

  TransportRxLen = 0;
  UART_read(uartHandle, npiRxBuf, NPI_UART_MSG_SOF_LEN);

  
  NPIUtil_ExitCS(key);
}


// -----------------------------------------------------------------------------
//! \brief      This routine writes copies buffer addr to the transport layer.
//!
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint16_t - number of bytes written to transport
// -----------------------------------------------------------------------------
uint16_t NPITLUART_writeTransport(uint16_t len)
{
  _npiCSKey_t key;
  key = NPIUtil_EnterCS();
  
  npiTxBuf[NPI_UART_MSG_SOF_IDX] = NPI_UART_MSG_SOF;
  npiTxBuf[len + 1] = NPITLUART_calcFCS((uint8_t *)&npiTxBuf[1],len);
  TransportTxLen = len + 2;

#ifdef POWER_SAVING
  TxActive = TRUE;

  // Start reading prior to impending write transaction
  // We can only call UART_write() once MRDY has been signaled from Master
  // device
  NPITLUART_readTransport();
#else
  // Check to see if transport is successful. If not, reset TxLen to allow
  // another write to be processed
  if(UART_write(uartHandle, npiTxBuf, TransportTxLen) == UART_ERROR)
  {
    len = 0;
  }
#endif //POWER_SAVING

  NPIUtil_ExitCS(key);
  
  return len;
}

// -----------------------------------------------------------------------------
//! \brief      Check for whether a complete and valid packet has been received.
//!             If packet incomplete do nothing. If packet is invalid, flush
//!             the Rx buffer
//!
//! \return     uint8_t - NPI_SUCCESS if Success, NPI_INCOMPLETE_PACKET if not
//!                       enough bytes recieved, NPI_INVALID_PACKET if incorrect 
//!                       format or FCS
// -----------------------------------------------------------------------------
uint8_t NPITLUART_validPacketFound(void)
{
  uint16_t payloadLen;
  uint8_t fcs;
  
  // SOF has already been removed from npiRxBuf
  payloadLen = (uint16) npiRxBuf[0];
  payloadLen += ((uint16) npiRxBuf[1]) << 8;
  
  // Check to make sure we have received all bytes of this message
  if (TransportRxLen < (payloadLen + NPI_UART_MSG_NON_PAYLOAD_LEN))
  {
    return NPI_INCOMPLETE_PKT;
  }
  
  // Calculate FCS of this message
  fcs = NPITLUART_calcFCS((uint8_t *)npiRxBuf, payloadLen + NPI_UART_MSG_HDR_LEN);

  if (fcs != npiRxBuf[payloadLen + NPI_UART_MSG_HDR_LEN])
  {
    // Invalid FCS, Flush RX buffer before returning error
    TransportRxLen = 0;
    
    return NPI_INVALID_PKT;
  }
  
  return NPI_SUCCESS;
}

// -----------------------------------------------------------------------------
//! \brief      Calculate FCS over the given length of buf
//!
//! \param[in]  buf - Pointer to first byte to use for FCS
//!             len - Number of bytes to calculate FCS over.
//!
//! \return     uint8_t - FCS value
// -----------------------------------------------------------------------------
uint8_t NPITLUART_calcFCS(uint8_t *buf, uint16_t len)
{   
    uint16_t i;
    uint8_t fcs = 0;
    
    for (i = 0; i < len; i++)
    {
        fcs ^= buf[i];
    }
    
    return fcs;
}
